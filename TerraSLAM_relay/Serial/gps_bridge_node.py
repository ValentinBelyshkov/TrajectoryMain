#!/usr/bin/env python3
"""
ROS 2 GPS Bridge Node (Pose/PoseStamped → GPS via calib.gpc → u-blox NMEA UART)
With watchdog:
  - Pose lost >1s: hold last coord for 3s, then stop TX.
  - 5 consecutive GPS jumps >50m: navigation broken, immediate stop TX.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
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
                if line.startswith("proj="):
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
            raise ValueError(f"Need ≥3 calib points, got {len(pts)}")

        A = np.array([[p[0], p[1], p[2], 1.0] for p in pts])
        lats = np.array([p[4] for p in pts])
        lons = np.array([p[3] for p in pts])
        alts = np.array([p[5] for p in pts])

        self.coeff_lat, *_ = np.linalg.lstsq(A, lats, rcond=None)
        self.coeff_lon, *_ = np.linalg.lstsq(A, lons, rcond=None)

        if np.std(alts) < 1e-6:
            self.use_z_for_alt = True
            self.base_alt = float(np.mean(alts))
        else:
            self.use_z_for_alt = False
            self.coeff_alt, *_ = np.linalg.lstsq(A, alts, rcond=None)

    def transform(self, x: float, y: float, z: float):
        v = np.array([x, y, z, 1.0])
        lat = float(np.dot(self.coeff_lat, v))
        lon = float(np.dot(self.coeff_lon, v))
        alt = (z + self.base_alt) if self.use_z_for_alt else float(np.dot(self.coeff_alt, v))
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


# =========================================================
# ROS Node
# =========================================================
class GpsBridgeNode(Node):
    def __init__(self):
        super().__init__("gps_bridge")

        # ── Parameters ──
        self.declare_parameter("calib_file", "calib.gpc")
        self.declare_parameter("pose_topic", "/camera_pose")
        self.declare_parameter("pose_type", "pose")
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("fix_type", 1)
        self.declare_parameter("sats", 12)
        self.declare_parameter("hdop", 1.0)
        self.declare_parameter("pose_timeout_s", 1.0)
        self.declare_parameter("hold_time_s", 3.0)
        self.declare_parameter("max_jump_m", 50.0)          # порог прыжка GPS
        self.declare_parameter("max_consecutive_jumps", 5)  # сколько подряд — тревога

        calib_file = self.get_parameter("calib_file").value
        pose_topic = self.get_parameter("pose_topic").value
        pose_type = self.get_parameter("pose_type").value
        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.max_rate = self.get_parameter("max_rate_hz").value
        self.fix_type = self.get_parameter("fix_type").value
        self.sats = self.get_parameter("sats").value
        self.hdop = self.get_parameter("hdop").value
        self.pose_timeout = self.get_parameter("pose_timeout_s").value
        self.hold_time = self.get_parameter("hold_time_s").value
        self.max_jump_m = self.get_parameter("max_jump_m").value
        self.max_consecutive_jumps = self.get_parameter("max_consecutive_jumps").value

        # ── Stats ──
        self.pose_count = 0
        self.sent_count = 0
        self.dropped_count = 0
        self.queue_full_count = 0
        self.last_status_time = 0.0

        # ── Watchdog / safety state ──
        self.last_pose_time = 0.0
        self.last_coords = None          # (lat, lon, alt) for hold
        self.emergency_logged = False
        self.emergency_start_time = None
        self.transmission_stopped = False

        # ── Jump detection ──
        self.consecutive_jumps = 0
        self.navigation_broken = False

        # ── Calibration ──
        try:
            self.calib = CalibTransform(calib_file)
            self.get_logger().info(f"✅ Calibration loaded: {calib_file}")
            self.get_logger().info(f"   Projection: {self.calib.projection}")
        except Exception as e:
            self.get_logger().error(f"❌ Calibration failed: {e}")
            raise

        # ── Serial ──
        self.ser = None
        self._open_serial()

        # ── State for speed/course ──
        self.prev_lat = None
        self.prev_lon = None
        self.prev_time = None
        self.last_send_time = 0.0

        # ── Comm thread ──
        self.send_queue = queue.Queue(maxsize=20)
        self.running = True
        self.comm_thread = threading.Thread(target=self._comm_loop, daemon=True)
        self.comm_thread.start()

        # ── Subscription ──
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        )

        if pose_type == "pose_stamped":
            self.create_subscription(PoseStamped, pose_topic, self.on_pose_stamped, qos)
            self.get_logger().info(f"📡 Subscribed to PoseStamped: {pose_topic}")
        elif pose_type == "pose":
            self.create_subscription(Pose, pose_topic, self.on_pose, qos)
            self.get_logger().info(f"📡 Subscribed to Pose: {pose_topic}")
        else:
            raise ValueError(f"Unknown pose_type: {pose_type}. Use 'pose' or 'pose_stamped'")

        self.get_logger().info(
            f"   → UART: {self.port} @ {self.baudrate} (max {self.max_rate} Hz)"
        )
        self.get_logger().info(
            f"   ⏱️ Watchdog: timeout={self.pose_timeout}s hold={self.hold_time}s"
        )
        self.get_logger().info(
            f"   🛡️ Jump guard: >{self.max_jump_m}m × {self.max_consecutive_jumps} times"
        )
        self.get_logger().info("⏳ Waiting for Pose messages...")

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1, write_timeout=0.5)
            self.get_logger().info(f"🔌 Serial opened {self.port} @ {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial open error: {e}")
            self.ser = None

    def _log_status(self, force=False):
        now = time.time()
        if force or (now - self.last_status_time) >= 2.0:
            self.last_status_time = now
            if self.navigation_broken:
                state = "NAV_BROKEN"
            elif self.transmission_stopped:
                state = "STOPPED"
            elif self.emergency_logged:
                state = "HOLDING"
            else:
                state = "OK"
            self.get_logger().info(
                f"📊 STATUS | poses={self.pose_count} sent={self.sent_count} "
                f"dropped={self.dropped_count} queue_full={self.queue_full_count} "
                f"jumps={self.consecutive_jumps} state={state}"
            )

    def _reset_watchdog(self):
        """Call when valid pose is received to clear emergency state."""
        if self.transmission_stopped or self.emergency_logged:
            self.get_logger().info("✅ Pose restored, resuming GPS transmission")
        self.last_pose_time = time.time()
        self.emergency_logged = False
        self.emergency_start_time = None
        self.transmission_stopped = False

    def _drain_queue(self):
        while not self.send_queue.empty():
            try:
                self.send_queue.get_nowait()
            except queue.Empty:
                break

    def _process_pose(self, x: float, y: float, z: float):
        now = time.time()
        self._reset_watchdog()

        # Rate limiter
        if (now - self.last_send_time) < (1.0 / self.max_rate):
            self.dropped_count += 1
            if self.pose_count % 50 == 0:
                self.get_logger().debug(f"⏱️ Pose #{self.pose_count} dropped (rate limit)")
            self._log_status()
            return

        self.pose_count += 1
        self.get_logger().debug(f"🎯 Pose #{self.pose_count} raw: x={x:.3f} y={y:.3f} z={z:.3f}")

        lat, lon, alt = self.calib.transform(x, y, z)
        self.get_logger().debug(f"🌍 Transformed: lat={lat:.7f} lon={lon:.7f} alt={alt:.2f}")

        # ── Jump detection (compare to last accepted coordinate) ──
        if self.prev_lat is not None and self.prev_lon is not None:
            dlat = math.radians(lat - self.prev_lat)
            dlon = math.radians(lon - self.prev_lon)
            a = (math.sin(dlat / 2) ** 2 +
                 math.cos(math.radians(self.prev_lat)) *
                 math.cos(math.radians(lat)) *
                 math.sin(dlon / 2) ** 2)
            dist_m = 2 * 6371000 * math.asin(math.sqrt(a))

            if dist_m > self.max_jump_m:
                self.consecutive_jumps += 1
                self.get_logger().warn(
                    f"⚠️ GPS jump #{self.consecutive_jumps}: {dist_m:.1f}m "
                    f"(>{self.max_jump_m}m from prev accepted)"
                )
                if self.consecutive_jumps >= self.max_consecutive_jumps:
                    if not self.navigation_broken:
                        self.navigation_broken = True
                        self.transmission_stopped = True
                        self.get_logger().error(
                            f"🚨 NAVIGATION BROKEN: {self.consecutive_jumps} consecutive "
                            f"jumps >{self.max_jump_m}m. Stopping GPS transmission."
                        )
                        self._drain_queue()
                self._log_status()
                return  # do NOT update prev_lat / prev_time
            else:
                if self.consecutive_jumps > 0:
                    self.get_logger().info(
                        f"✅ GPS stabilized after {self.consecutive_jumps} jump(s)"
                    )
                self.consecutive_jumps = 0
                if self.navigation_broken:
                    self.navigation_broken = False
                    self.transmission_stopped = False
                    self.get_logger().info("✅ Navigation recovered, resuming GPS transmission")

        # If still broken (should not happen here, but safety guard)
        if self.navigation_broken:
            return

        # ── Speed & course ──
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
        self.last_coords = (lat, lon, alt)

        # ── Build NMEA ──
        gga = build_gga(lat, lon, alt, self.fix_type, self.sats, self.hdop)
        rmc = build_rmc(lat, lon, speed_kn, course, self.fix_type)
        nmea_payload = (gga + "\r\n" + rmc + "\r\n").encode("ascii")

        self.get_logger().debug(
            f"📤 NMEA ready: {gga[:35]}... + {rmc[:35]}... ({len(nmea_payload)} bytes)"
        )

        try:
            if not self.send_queue.full():
                self.send_queue.put_nowait(nmea_payload)
                self.get_logger().debug(f"✅ Queued (size={self.send_queue.qsize()})")
            else:
                self.queue_full_count += 1
                self.get_logger().warn(
                    f"⚠️ Queue full! Dropping pose #{self.pose_count}"
                )
        except queue.Full:
            self.queue_full_count += 1
            self.get_logger().warn("⚠️ Queue full (exception)")

        self._log_status()

    def on_pose(self, msg: Pose):
        self._process_pose(msg.position.x, msg.position.y, msg.position.z)

    def on_pose_stamped(self, msg: PoseStamped):
        self._process_pose(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def _inject_hold_packet(self):
        """Send last known coordinate with zero speed/course."""
        if self.last_coords is None:
            return
        lat, lon, alt = self.last_coords
        gga = build_gga(lat, lon, alt, self.fix_type, self.sats, self.hdop)
        rmc = build_rmc(lat, lon, 0.0, 0.0, self.fix_type)
        packet = (gga + "\r\n" + rmc + "\r\n").encode("ascii")
        try:
            if not self.send_queue.full():
                self.send_queue.put_nowait(packet)
                self.get_logger().debug("🔄 Injected HOLD packet (last coord, zero speed)")
        except queue.Full:
            pass

    def _comm_loop(self):
        self.get_logger().info("🚀 Comm thread started")
        while self.running:
            try:
                if self.ser is None or not self.ser.is_open:
                    self.get_logger().warn("🔌 Serial not ready, retrying in 1s...")
                    time.sleep(1.0)
                    self._open_serial()
                    continue

                # ── Watchdog logic (pose lost) ──
                now = time.time()
                dt = now - self.last_pose_time

                if dt > self.pose_timeout and not self.emergency_logged:
                    self.get_logger().error(
                        f"🚨 EMERGENCY: Pose lost for {dt:.1f}s! "
                        f"Holding last coordinate for {self.hold_time}s..."
                    )
                    self.emergency_logged = True
                    self.emergency_start_time = now

                if self.emergency_logged and not self.transmission_stopped:
                    elapsed = now - self.emergency_start_time
                    if elapsed < self.hold_time:
                        if (now - self.last_send_time) >= (1.0 / self.max_rate):
                            self._inject_hold_packet()
                            self.last_send_time = now
                    else:
                        self.get_logger().error(
                            "🛑 Pose lost > hold_time. GPS transmission STOPPED."
                        )
                        self.transmission_stopped = True

                # ── Normal TX from queue ──
                if self.send_queue.empty():
                    time.sleep(0.005)
                    continue

                packet = self.send_queue.get_nowait()
                written = self.ser.write(packet)
                self.sent_count += 1
                self.get_logger().debug(
                    f"🔵 UART TX: {written} bytes | total_sent={self.sent_count}"
                )

            except serial.SerialTimeoutException:
                self.get_logger().error("⏱️ Serial write timeout!")
            except Exception as e:
                self.get_logger().error(f"💥 Comm error: {e}")
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(2.0)

        self.get_logger().info("🏁 Comm thread stopped")

    def destroy_node(self):
        self.get_logger().info("🛑 Shutting down...")
        self.running = False
        self.comm_thread.join(timeout=2.0)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info("🔌 Serial closed")
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
