#!/usr/bin/env python3
import json
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

# File that system_manager reads via GET /api/v1/slam/status
STATUS_FILE = "/tmp/terraslam_slam_status"

# Correspondence of ORB-SLAM3 int codes to string state names.
TRACKING_STATE_NAMES = {
    -1: "UNKNOWN",
    0:  "NO_IMAGES_YET",
    1:  "NOT_INITIALIZED",
    2:  "OK",
    3:  "TRACKING_LOST",
}

# Filtered pose topic produced by the gps_filter component. We consume the
# already-filtered stream so the UI status matches what reaches the hardware.
FILTERED_POSE_TOPIC = "/orb_slam3/robot_pose_slam_filtered"


class GcpRelay(Node):
    def __init__(self, path):
        super().__init__('gcp_relay')
        # NOTE: this node no longer publishes /camera/gps — the gps_bridge
        # component owns that ROS topic now. This node only derives the
        # SLAM tracking status written to STATUS_FILE (consumed by
        # GET /api/v1/slam/status).
        self.create_subscription(PoseStamped, FILTERED_POSE_TOPIC, self.on_pose, 10)

        # Internal state for status reporting.
        self._tracking_state: int = -1   # from /orb_slam3/slam_info (unused here)
        self._pose_state: int = -1       # derived from Pose values
        self._last_pose_ts: float = 0.0  # time of last pose message

        # Timer: write status once per second.
        self.create_timer(1.0, self._write_status)

        pts = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line[0] in ('#', '+'):
                    continue
                vals = line.split()
                if len(vals) < 5:
                    continue
                px, py, pz = float(vals[0]), float(vals[1]), float(vals[2])
                lon, lat = float(vals[3]), float(vals[4])
                alt = float(vals[5]) if len(vals) >= 6 else 0.0
                pts.append((px, py, lat, lon, alt))

        if len(pts) < 3:
            raise ValueError(f"Need at least 3 GCP points, loaded {len(pts)}")

        self.get_logger().info(f"Loaded {len(pts)} calibration points")

    def on_pose(self, msg: PoseStamped):
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        self._last_pose_ts = time.time()

        # Decode special pose values -> tracking state.
        if abs(x + 3.0) < 0.01 and abs(y + 3.0) < 0.01 and abs(z + 3.0) < 0.01:
            self._pose_state = 3   # TRACKING_LOST
            return
        if abs(x + 1.0) < 0.01 and abs(y + 1.0) < 0.01 and abs(z + 1.0) < 0.01:
            self._pose_state = 1   # NOT_INITIALIZED
            return
        if abs(x) < 0.01 and abs(y) < 0.01 and abs(z) < 0.01:
            self._pose_state = 0   # NO_IMAGES_YET
            return

        self._pose_state = 2   # OK

    def _write_status(self):
        """Write SLAM state to /tmp/terraslam_slam_status once per second."""
        state = self._tracking_state if self._tracking_state != -1 else self._pose_state
        state_name = TRACKING_STATE_NAMES.get(state, "UNKNOWN")
        initialized = (state == 2)

        # If no pose has arrived for a while, flag NO_IMAGES.
        if self._last_pose_ts > 0 and (time.time() - self._last_pose_ts) > 5.0:
            state = 0
            state_name = "NO_IMAGES_YET"
            initialized = False

        status = {
            "slam_state":      state,
            "slam_state_name": state_name,
            "initialized":     initialized,
            "drone_mode":      "LOCALIZATION_ONLY",
            "timestamp":       time.time(),
        }

        try:
            with open(STATUS_FILE, "w") as f:
                json.dump(status, f)
        except Exception as e:
            self.get_logger().warn(f"Failed to write status file: {e}")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GcpRelay(sys.argv[1] if len(sys.argv) > 1 else 'calib.gpc'))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
