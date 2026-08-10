#!/usr/bin/env python3
"""
ROS 2 GPS Bridge Node (Pose/PoseStamped -> GPS via calib.gpc -> u-blox NMEA UART)

DUMB BRIDGE (post-refactor): this component only
  - transforms the (already filtered) pose to GPS via calib.gpc,
  - PUBLISHES the GPS as NavSatFix on /camera/gps (ROS),
  - sends NMEA to the flight-controller UART.

All filtering / safety logic (jump detection, navigation-broken, pose-lost
watchdog) now lives in the separate `gps_filter` component, which republishes
a filtered pose on /orb_slam3/robot_pose_slam_filtered. When that pose is the
lost sentinel (-3,-3,-3) this bridge simply stops transmitting.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
import queue
import threading
import time
import math
from datetime import datetime, timezone

try:
    import numpy as np
except ImportError:
    raise RuntimeError("numpy required: pip install numpy")

try:
    import serial
except ImportError:
    raise RuntimeError("pyserial required: pip install pyserial")


# =========================================================
# Calibration parser & affine transform
# =========================================================
class CalibTransform:
    def __init__(self, filepath: str):
        self.coeff_lat = None
        self.coeff_lon = None
        self.use_z_for_alt = True
        self.base_alt = 0.0
        self.projection = None
        self._load(filepath)

    def _load(self, filepath: str):
        pts = []
        with open(filepath, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                if line.startswith("+") or line.startswith("proj="):
                    self.projection = line
                    continue
                tok = line.split()
                if len(tok) >= 5:
                    x, y, z = float(tok[0]), float(tok[1]), float(tok[2])
                    lon = float(tok[3])
                    lat = float(tok[4])
                    alt = float(tok[5]) if len(tok) > 5 else 0.0
                    pts.append((x, y, z, lon, lat, alt))

        if len(pts) < 3:
            raise ValueError(f"Need >=3 calib points, got {len(pts)}")

        # 2D affine [x, y, 1] -- calib.gpc has z ~constant, so a 4D
        # [x,y,z,1] fit is ill-conditioned and explodes GPS with tiny z
        # changes. Use x,y only (matches the working gps_client.py).
        A = np.array([[p[0], p[1], 1.0] for p in pts])
        lats = np.array([p[4] for p in pts])
        lons = np.array([p[3] for p in pts])
        alts = np.array([p[5] for p in pts])

        self.coeff_lat, *_ = np.linalg.lstsq(A, lats, rcond=None)
        self.coeff_lon, *_ = np.linalg.lstsq(A, lons, rcond=None)

        self.use_z_for_alt = True
        self.base_alt = float(np.mean(alts))

    def transform(self, x: float, y: float, z: float):
        v = np.array([x, y, 1.0])
        lat = float(np.dot(self.coeff_lat, v))
        lon = float(np.dot(self.coeff_lon, v))
        alt = self.base_alt
        return lat, lon, alt


# =========================================================
# NMEA builders
# =========================================================
def _checksum(body: str) -> str:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"{cs:02X}"

def _fmt_lat(lat: float):
    d = int(abs(lat))
    m = (abs(lat) - d) * 60.0
    return f"{d:02d}{m:09.6f}", ("N" if lat >= 0 else "S")

def _fmt_lon(lon: float):
    d = int(abs(lon))
    m = (abs(lon) - d) * 60.0
    return f"{d:03d}{m:09.6f}", ("E" if lon >= 0 else "W")

def build_gga(lat, lon, alt, fix, sats, hdop):
    t = datetime.now(timezone.utc)
    lat_s, lat_d = _fmt_lat(lat)
    lon_s, lon_d = _fmt_lon(lon)
    body = (f"GNGGA,{t.strftime('%H%M%S')}.{t.microsecond // 100000:01d},"
            f"{lat_s},{lat_d},{lon_s},{lon_d},"
            f"{fix},{sats:02d},{hdop:.1f},{alt:.1f},M,0.0,M,,")
    return f"${body}*{_checksum(body)}"

def build_rmc(lat, lon, speed_kn, course, fix):
    t = datetime.now(timezone.utc)
    lat_s, lat_d = _fmt_lat(lat)
    lon_s, lon_d = _fmt_lon(lon)
    status = "A" if fix > 0 else "V"
    body = (f"GNRMC,{t.strftime('%H%M%S')}.{t.microsecond // 100000:01d},{status},"
            f"{lat_s},{lat_d},{lon_s},{lon_d},"
            f"{speed_kn:.1f},{course:.1f},{t.strftime('%d%m%y')},,,A")
    return f"${body}*{_checksum(body)}"


# Sentinel pose (tracking/transmission lost) published by gps_filter.
SENTINEL = (-3.0, -3.0, -3.0)


# =========================================================
# ROS Node
# =========================================================
class GpsBridgeNode(Node):
    def __init__(self):
        super().__init__("gps_bridge")

        # ---- Parameters ----
        self.declare_parameter("calib_file", "calib.gpc")
        self.declare_parameter("pose_topic", "/orb_slam3/robot_pose_slam_filtered")
        self.declare_parameter("pose_type", "pose_stamped")
        self.declare_parameter("gps_topic", "/camera/gps")
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("fix_type", 1)
        self.declare_parameter("sats", 12)
        self.declare_parameter("hdop", 1.0)

        calib_file = self.get_parameter("calib_file").value
        pose_topic = self.get_parameter("pose_topic").value
        pose_type = self.get_parameter("pose_type").value
        gps_topic = self.get_parameter("gps_topic").value
        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.max_rate = self.get_parameter("max_rate_hz").value
        self.fix_type = self.get_parameter("fix_type").value
        self.sats = self.get_parameter("sats").value
        self.hdop = self.get_parameter("hdop").value

        # ---- Stats ----
        self.pose_count = 0
        self.sent_count = 0
        self.dropped_count = 0
        self.queue_full_count = 0
        self.last_status_time = 0.0

        # ---- Calibration ----
        try:
            self.calib = CalibTransform(calib_file)
            self.get_logger().info(f"Calibration loaded: {calib_file}")
        except Exception as e:
            self.get_logger().error(f"Calibration failed: {e}")
            raise

        # ---- Serial ----
        self.ser = None
        self._open_serial()

        # ---- State for speed/course ----
        self.prev_lat = None
        self.prev_lon = None
        self.prev_time = None
        self.last_send_time = 0.0
        self.tx_stopped = False

        # ---- ROS publisher (GPS -> ROS) ----
        self.gps_pub = self.create_publisher(NavSatFix, gps_topic, 10)
        self.get_logger().info(f"Publishing GPS to ROS topic: {gps_topic}")

        # ---- Comm thread ----
        self.send_queue = queue.Queue(maxsize=20)
        self.running = True
        self.comm_thread = threading.Thread(target=self._comm_loop, daemon=True)
        self.comm_thread.start()

        # ---- Subscription (filtered pose) ----
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        )
        if pose_type == "pose_stamped":
            self.create_subscription(PoseStamped, pose_topic, self.on_pose_stamped, qos)
            self.get_logger().info(f"Subscribed to PoseStamped: {pose_topic}")
        elif pose_type == "pose":
            self.create_subscription(Pose, pose_topic, self.on_pose, qos)
            self.get_logger().info(f"Subscribed to Pose: {pose_topic}")
        else:
            raise ValueError(f"Unknown pose_type: {pose_type}. Use 'pose' or 'pose_stamped'")

        self.get_logger().info(
            f"UART: {self.port} @ {self.baudrate} (max {self.max_rate} Hz)"
        )
        self.get_logger().info("Waiting for filtered pose messages...")

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1, write_timeout=0.5)
            self.get_logger().info(f"Serial opened {self.port} @ {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Serial open error: {e}")
            self.ser = None

    def _is_sentinel(self, x, y, z):
        return (
            abs(x - SENTINEL[0]) < 0.01
            and abs(y - SENTINEL[1]) < 0.01
            and abs(z - SENTINEL[2]) < 0.01
        )

    def _log_status(self, force=False):
        now = time.time()
        if force or (now - self.last_status_time) >= 2.0:
            self.last_status_time = now
            state = "STOPPED" if self.tx_stopped else "OK"
            self.get_logger().info(
                f"STATUS | poses={self.pose_count} sent={self.sent_count} "
                f"dropped={self.dropped_count} queue_full={self.queue_full_count} state={state}"
            )

    def _process_pose(self, x, y, z):
        # Lost sentinel from the filter -> stop transmitting, do not publish GPS.
        if self._is_sentinel(x, y, z):
            if not self.tx_stopped:
                self.tx_stopped = True
                self.get_logger().error("Lost sentinel received -> stopping GPS transmission")
                self._drain_queue()
            self._log_status()
            return

        if self.tx_stopped:
            self.tx_stopped = False
            self.get_logger().info("Valid pose received -> resuming GPS transmission")

        now = time.time()

        # Rate limiter
        if (now - self.last_send_time) < (1.0 / self.max_rate):
            self.dropped_count += 1
            self._log_status()
            return

        self.pose_count += 1
        lat, lon, alt = self.calib.transform(x, y, z)

        # ---- Publish to ROS ----
        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = "map"
        gps.status.status = NavSatStatus.STATUS_FIX
        gps.latitude = lat
        gps.longitude = lon
        gps.altitude = alt
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.gps_pub.publish(gps)

        # ---- Speed & course ----
        speed_kn = 0.0
        course = 0.0
        if self.prev_lat is not None and self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0:
                dlat = math.radians(lat - self.prev_lat)
                dlon = math.radians(lon - self.prev_lon)
                a = (math.sin(dlat / 2) ** 2 +
                     math.cos(math.radians(self.prev_lat)) *
                     math.cos(math.radians(lat)) *
                     math.sin(dlon / 2) ** 2)
                dist_m = 2 * 6371000 * math.asin(math.sqrt(a))
                speed_ms = dist_m / dt
                speed_kn = speed_ms * 1.94384
                yc = math.sin(dlon) * math.cos(math.radians(lat))
                xc = (math.cos(math.radians(self.prev_lat)) * math.sin(math.radians(lat)) -
                      math.sin(math.radians(self.prev_lat)) * math.cos(math.radians(lat)) * math.cos(dlon))
                course = (math.degrees(math.atan2(yc, xc)) + 360.0) % 360.0

        self.prev_lat = lat
        self.prev_lon = lon
        self.prev_time = now
        self.last_send_time = now

        # ---- Build NMEA ----
        gga = build_gga(lat, lon, alt, self.fix_type, self.sats, self.hdop)
        rmc = build_rmc(lat, lon, speed_kn, course, self.fix_type)
        nmea_payload = (gga + "\r\n" + rmc + "\r\n").encode("ascii")

        try:
            if not self.send_queue.full():
                self.send_queue.put_nowait(nmea_payload)
            else:
                self.queue_full_count += 1
                self.get_logger().warn("Queue full! Dropping pose")
        except queue.Full:
            self.queue_full_count += 1

        self._log_status()

    def on_pose(self, msg: Pose):
        self._process_pose(msg.position.x, msg.position.y, msg.position.z)

    def on_pose_stamped(self, msg: PoseStamped):
        self._process_pose(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _drain_queue(self):
        while not self.send_queue.empty():
            try:
                self.send_queue.get_nowait()
            except queue.Empty:
                break

    def _comm_loop(self):
        self.get_logger().info("Comm thread started")
        while self.running:
            try:
                if self.ser is None or not self.ser.is_open:
                    self.get_logger().warn("Serial not ready, retrying in 1s...")
                    time.sleep(1.0)
                    self._open_serial()
                    continue

                if self.send_queue.empty():
                    time.sleep(0.005)
                    continue

                packet = self.send_queue.get_nowait()
                written = self.ser.write(packet)
                self.sent_count += 1
            except serial.SerialTimeoutException:
                self.get_logger().error("Serial write timeout!")
            except Exception as e:
                self.get_logger().error(f"Comm error: {e}")
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(2.0)

        self.get_logger().info("Comm thread stopped")

    def destroy_node(self):
        self.get_logger().info("Shutting down...")
        self.running = False
        self.comm_thread.join(timeout=2.0)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("Serial closed")
            except Exception as e:
                self.get_logger().error(f"Error closing serial: {e}")
        self._log_status(force=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
