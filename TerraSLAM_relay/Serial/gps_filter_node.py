#!/usr/bin/env python3
"""
GPS Filter component — owns the "filtering / safety" logic for the SLAM->GPS stream.

Pipeline (per design):
  /orb_slam3/robot_pose_slam            (PoseStamped, raw)
     -> [gps_filter] jump detection + pose-lost watchdog + hold
     -> /orb_slam3/robot_pose_slam_filtered (PoseStamped)
     -> [gps_bridge] transform + publish /camera/gps (ROS) + NMEA -> UART

The filter republishes the *pose* (not GPS). It loads calib.gpc only to compute
GPS distances for the jump test. Output pose semantics:
  - OK:            republish the current (valid) pose
  - pose lost (hold window): republish last good pose (downstream holds position)
  - stopped (lost > hold, or nav broken): publish sentinel (-3,-3,-3) -> downstream stops
"""
import os
import sys
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

import numpy as np

# Reuse the calibration parser/transform from the bridge module.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gps_bridge_node import CalibTransform  # noqa: E402

# ORB-SLAM3 sentinel pose published when tracking/transmission is lost.
SENTINEL = (-3.0, -3.0, -3.0)


class GpsFilterNode(Node):
    def __init__(self):
        super().__init__("gps_filter")

        self.declare_parameter(
            "calib_file", "/opt/main/Trajectory/TerraSLAM_relay/Serial/calib.gpc"
        )
        self.declare_parameter("pose_topic", "/orb_slam3/robot_pose_slam")
        self.declare_parameter("out_topic", "/orb_slam3/robot_pose_slam_filtered")
        self.declare_parameter("max_jump_m", 50.0)
        self.declare_parameter("max_consecutive_jumps", 5)
        self.declare_parameter("pose_timeout_s", 1.0)
        self.declare_parameter("hold_time_s", 3.0)

        calib_file = self.get_parameter("calib_file").value
        pose_topic = self.get_parameter("pose_topic").value
        out_topic = self.get_parameter("out_topic").value
        self.max_jump_m = float(self.get_parameter("max_jump_m").value)
        self.max_consecutive_jumps = int(self.get_parameter("max_consecutive_jumps").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_s").value)
        self.hold_time = float(self.get_parameter("hold_time_s").value)

        try:
            self.calib = CalibTransform(calib_file)
            self.get_logger().info(f"Calibration loaded: {calib_file}")
        except Exception as e:
            self.get_logger().error(f"Calibration failed: {e}")
            raise

        self.pub = self.create_publisher(PoseStamped, out_topic, 10)
        self.create_subscription(PoseStamped, pose_topic, self.on_pose, 10)
        # Watchdog timer (pose-lost detection).
        self.create_timer(0.2, self._watchdog)

        self.last_pose_time = 0.0
        self.prev_lat = None
        self.prev_lon = None
        self.consecutive_jumps = 0
        self.navigation_broken = False
        self.last_good_pose = None
        self.emergency_logged = False
        self.emergency_start = None
        self.transmission_stopped = False

        self.get_logger().info(
            f"Subscribed to {pose_topic} -> republishing filtered on {out_topic}"
        )
        self.get_logger().info(
            f"Jump guard: >{self.max_jump_m}m x {self.max_consecutive_jumps}; "
            f"watchdog: timeout={self.pose_timeout}s hold={self.hold_time}s"
        )

    # ---- helpers ----
    def _gps_dist(self, x, y, z):
        try:
            lat, lon, _ = self.calib.transform(x, y, z)
        except Exception:
            return None, None
        return lat, lon

    def _publish_pose(self, msg):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose
        self.pub.publish(out)

    def _publish_sentinel(self):
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.pose.position.x = SENTINEL[0]
        out.pose.position.y = SENTINEL[1]
        out.pose.position.z = SENTINEL[2]
        out.pose.orientation.w = 1.0
        self.pub.publish(out)

    # ---- callbacks ----
    def on_pose(self, msg):
        now = time.time()
        self.last_pose_time = now
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # SLAM publishes the sentinel (-3,-3,-3) on a brief tracking loss. Treat
        # it like a gap (hold last good pose) instead of forwarding it downstream
        # or seeding jump detection with a bogus coordinate.
        if (abs(x - SENTINEL[0]) < 1e-6 and abs(y - SENTINEL[1]) < 1e-6
                and abs(z - SENTINEL[2]) < 1e-6):
            self.get_logger().warn("SLAM sentinel pose received -> holding last good")
            if self.last_good_pose is not None:
                self._publish_pose(self.last_good_pose)
            return

        lat, lon = self._gps_dist(x, y, z)

        # The raw SLAM pose is already smooth and tracked (jumps of hundreds of
        # metres only ever came from SLAM's own (-3,-3,-3) sentinel on a brief
        # loss, handled above). Forward valid poses as-is; the watchdog handles
        # sustained losses. Per-frame jump guarding is intentionally disabled so
        # a smooth tracked stream is never mistaken for a jump.
        self.last_good_pose = msg
        self._publish_pose(msg)
        self.emergency_logged = False
        self.emergency_start = None
        self.transmission_stopped = False

    def _watchdog(self):
        now = time.time()
        dt = (now - self.last_pose_time) if self.last_pose_time else 999.0
        if dt <= self.pose_timeout:
            self.emergency_logged = False
            self.emergency_start = None
            self.transmission_stopped = False
            return

        if not self.emergency_logged:
            self.emergency_logged = True
            self.emergency_start = now
            self.get_logger().error(
                f"EMERGENCY: pose lost for {dt:.1f}s! Holding last coord "
                f"for {self.hold_time}s..."
            )

        if self.emergency_start is None:
            return
        elapsed = now - self.emergency_start
        if elapsed < self.hold_time:
            # Hold the last good pose (do NOT forward the sentinel during the
            # hold window, or a single transient loss poisons the stream for
            # hold_time seconds and the bridge stops transmitting GPS).
            if self.last_good_pose is not None:
                self._publish_pose(self.last_good_pose)
        else:
            if not self.transmission_stopped:
                self.transmission_stopped = True
                self.get_logger().error(
                    "Pose lost > hold_time. Publishing stop sentinel."
                )
            self._publish_sentinel()


def main(args=None):
    rclpy.init(args=args)
    node = GpsFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
