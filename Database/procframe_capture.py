"""Capture (frame, pose) pairs into a procframe directory for calibration.

This node replaces the old ORB-SLAM3 `MapControl command 5` behavior that the
previous wrapper used to write each tracked frame + its pose into the project's
procframe folder. The new ORB-SLAM3 ROS2 wrapper no longer exposes that
command, so we recover the same data on the backend side:

  * subscribe to /orb_slam3/robot_pose_slam (geometry_msgs/PoseStamped) — the
    current camera pose published by the wrapper. Per the wrapper sources
    (slam_node_base.cpp), this is published ONLY while tracking is active, so a
    fresh pose is itself a reliable "tracking is running" signal;
  * subscribe to /orb_slam3/slam_info — diagnostic info that includes
    tracking_frequency (>0 while tracking is active);
  * subscribe to the camera image topic (default /camera/image_raw/compressed,
    as published by image_publish.py) — the actual frame;
  * when the capture flag file (/tmp/terraslam_save_frames) exists AND tracking
    is active, write <stem>.jpg (the frame) and <stem>.txt (the pose "x y z")
    into procframe/.

Tracking is considered active when a pose has been received recently
(POSE_FRESHNESS_S, default 1.5 s) AND slam_info reports tracking_frequency > 0
(when slam_info is available). This avoids saving frames during tracking loss /
initialization, which would pollute the procframe set with invalid poses.

Run:
  python3 procframe_capture.py --procframe-dir /path/to/procframe
"""
import argparse
import os
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, Image
try:
    from slam_msgs.msg import SlamInfo
except Exception:  # slam_msgs may be unavailable in some workspaces
    SlamInfo = None

# How long (seconds) a pose may be stale before we consider tracking lost.
POSE_FRESHNESS_S = 1.5

# Topic carrying the wrapper's diagnostic info (tracking_frequency etc.).
SLAM_INFO_TOPIC = "/orb_slam3/slam_info"


def _now_ms() -> int:
    return int(time.time() * 1000)


class ProcFrameCapture(Node):
    def __init__(
        self,
        procframe_dir: str,
        image_topic: str = "/camera/image_raw/compressed",
        pose_topic: str = "/orb_slam3/robot_pose_slam",
        slam_info_topic: str = SLAM_INFO_TOPIC,
        flag_file: str = "/tmp/terraslam_save_frames",
        pose_freshness_s: float = POSE_FRESHNESS_S,
    ):
        super().__init__("procframe_capture")

        self.procframe_dir = procframe_dir
        self.flag_file = flag_file
        self.bridge = CvBridge()
        self.pose_freshness_s = float(pose_freshness_s)

        # Counter for output filenames (frame_000001.jpg / .txt).
        self.counter = 0
        self.saved = 0
        self.failed = 0
        self.last_report = time.time()

        # Counters of incoming messages (for diagnostics).
        self._pose_count = 0
        self._img_count = 0
        self._info_count = 0
        # Throttled logging of incoming poses.
        self._last_pose_log = 0.0

        # Latest pose latched from the pose topic. (stamp_ns, x, y, z)
        self._pose = None  # type: ignore
        # Wall-clock time (monotonic) when the last pose was received.
        self._pose_recv = 0.0

        # Tracking frequency from slam_info (None until first message arrives).
        # While None we rely solely on pose freshness; once known we also
        # require tracking_frequency > 0.
        self._tracking_freq = None  # type: ignore

        # Make sure procframe exists and is writable up front.
        try:
            os.makedirs(self.procframe_dir, exist_ok=True)
            probe = os.path.join(self.procframe_dir, ".write_test")
            with open(probe, "w") as fh:
                fh.write("ok")
            os.remove(probe)
            self.get_logger().info(f"procframe dir OK (writable): {self.procframe_dir}")
        except OSError as exc:
            self.get_logger().error(f"procframe dir NOT writable: {self.procframe_dir} ({exc})")

        # The ORB-SLAM3 wrapper publishes pose/slam_info under a topic whose
        # namespace depends on how the node is launched. In the current
        # unirobot.launch.py the node runs with an EMPTY namespace, so the pose
        # lands on `/robot_pose_slam` (NOT `/orb_slam3/robot_pose_slam`). To be
        # robust against both layouts we subscribe to BOTH the given topic and
        # its prefixed/unprefixed variant.
        self._pose_topics = self._topic_variants(pose_topic)
        for t in self._pose_topics:
            self.create_subscription(PoseStamped, t, self.pose_cb, 10)

        # slam_info subscriber (slam_msgs/msg/SlamInfo). Published only while
        # tracking is active; carries tracking_frequency (>0 while tracking).
        # Best-effort: if slam_msgs isn't built in this workspace we skip it.
        self._info_topics = []
        if SlamInfo is not None:
            self._info_topics = self._topic_variants(slam_info_topic)
            for t in self._info_topics:
                self.create_subscription(SlamInfo, t, self.slam_info_cb, 10)
        else:
            self.get_logger().warn(
                "slam_msgs/SlamInfo not available — tracking_frequency gate disabled"
            )

        # Image subscriber (compressed by default; fall back to raw if needed).
        if image_topic.endswith("/compressed") or image_topic.endswith(".compressed"):
            self.image_sub = self.create_subscription(
                CompressedImage, image_topic, self.image_cb, 10
            )
        else:
            self.image_sub = self.create_subscription(
                Image, image_topic, self.image_cb, 10
            )

        self.get_logger().info(
            f"ProcFrame capture: image={image_topic} "
            f"pose_topics={self._pose_topics} slam_info_topics={self._info_topics} "
            f"procframe={self.procframe_dir}"
        )

        # Heartbeat timer (1 Hz) for logs.
        self.timer = self.create_timer(1.0, self._report)

    @staticmethod
    def _topic_variants(topic: str):
        """Return [topic, prefixed/unprefixed-orb_slam3-variant] de-duplicated."""
        topic = topic.strip()
        if not topic.startswith("/"):
            topic = "/" + topic
        ns = "/orb_slam3"
        if topic.startswith(ns + "/"):
            alt = topic[len(ns):]                  # "/robot_pose_slam"
        else:
            alt = ns + topic                      # "/orb_slam3/robot_pose_slam"
        out = []
        for t in (topic, alt):
            if t not in out:
                out.append(t)
        return out

    def _capture_enabled(self) -> bool:
        return os.path.exists(self.flag_file)

    def _tracking_active(self) -> bool:
        # Pose is the authoritative "tracking is running" signal: the wrapper
        # publishes it only on successful tracked frames.
        if self._pose is None:
            return False
        if (time.monotonic() - self._pose_recv) > self.pose_freshness_s:
            return False
        # If slam_info is known, also require tracking_frequency > 0.
        if self._tracking_freq is not None and self._tracking_freq <= 0.0:
            return False
        return True

    def pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._pose = (stamp_ns, p.x, p.y, p.z)
        self._pose_recv = time.monotonic()
        self._pose_count += 1
        now = time.monotonic()
        if self._pose_count == 1 or (now - self._last_pose_log) > 2.0:
            self._last_pose_log = now
            self.get_logger().info(
                f"[DIAG] pose_cb #{self._pose_count}: "
                f"stamp_ns={stamp_ns} xyz=({p.x:.3f},{p.y:.3f},{p.z:.3f})"
            )

    def slam_info_cb(self, msg):
        # slam_info is slam_msgs/msg/SlamInfo (published only while tracking is
        # active); tracking_frequency > 0 means frames are being tracked.
        self._info_count += 1
        try:
            freq = float(msg.tracking_frequency)
            self._tracking_freq = freq
            self.get_logger().info(
                f"[DIAG] slam_info_cb #{self._info_count}: "
                f"tracking_frequency={freq} "
                f"num_keyframes={getattr(msg, 'num_keyframes_in_current_map', '?')}"
            )
        except Exception as exc:
            # Ignore malformed slam_info; keep last known value.
            self.get_logger().warn(
                f"[DIAG] slam_info_cb #{self._info_count}: parse error ({exc})"
            )

    def image_cb(self, msg):
        self._img_count += 1
        if not self._capture_enabled():
            return
        if not self._tracking_active():
            # No recent pose / tracking lost — skip. We only save pairs while
            # SLAM is actively tracking, so every stored frame has a valid pose.
            age = (time.monotonic() - self._pose_recv) if self._pose is not None else None
            self.get_logger().debug(
                f"[DIAG] image_cb #{self._img_count}: SKIP "
                f"(capture_enabled=True tracking_active=False "
                f"pose_count={self._pose_count} "
            f"pose_age={('None' if age is None else round(age, 3))}s "
                f"tracking_freq={self._tracking_freq})"
            )
            return

        try:
            if isinstance(msg, CompressedImage):
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.failed += 1
            self.get_logger().warn(f"Failed to decode image: {exc}")
            return

        _, x, y, z = self._pose
        self.counter += 1
        stem = f"frame_{self.counter:06d}"
        jpg_path = os.path.join(self.procframe_dir, f"{stem}.jpg")
        txt_path = os.path.join(self.procframe_dir, f"{stem}.txt")

        try:
            cv2.imwrite(jpg_path, cv_image)
            with open(txt_path, "w") as fh:
                fh.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
            self.saved += 1
            self.get_logger().info(
                f"[DIAG] image_cb #{self._img_count}: SAVED {stem} "
                f"xyz=({x:.3f},{y:.3f},{z:.3f})"
            )
        except OSError as exc:
            self.failed += 1
            self.get_logger().error(f"Failed to write procframe pair: {exc}")

    def _report(self):
        now = time.time()
        if now - self.last_report < 1.0:
            return
        self.last_report = now
        enabled = self._capture_enabled()
        tracking = self._tracking_active()
        age = (time.monotonic() - self._pose_recv) if self._pose is not None else None
        # Detailed reason why nothing is being saved.
        if enabled and not tracking:
            if self._pose is None:
                reason = "no pose received yet"
            elif age is not None and age > self.pose_freshness_s:
                reason = f"pose stale ({age:.2f}s > {self.pose_freshness_s}s)"
            elif self._tracking_freq is not None and self._tracking_freq <= 0.0:
                reason = f"tracking_frequency<=0 ({self._tracking_freq})"
            else:
                reason = "unknown"
        else:
            reason = "n/a"
        self.get_logger().info(
            f"[STATUS] capture_enabled={enabled} tracking_active={tracking} "
            f"reason='{reason}' pose_count={self._pose_count} "
            f"img_count={self._img_count} info_count={self._info_count} "
            f"pose_age={age if age is None else round(age, 3)}s "
            f"tracking_freq={self._tracking_freq} saved={self.saved} "
            f"failed={self.failed} last_stem=frame_{self.counter:06d}"
        )


def main(args=None):
    parser = argparse.ArgumentParser(description="Capture frames+poses into procframe")
    parser.add_argument("--procframe-dir", required=True, help="Output procframe directory")
    parser.add_argument(
        "--image-topic",
        default="/camera/image_raw/compressed",
        help="Camera image topic (compressed or raw)",
    )
    parser.add_argument(
        "--pose-topic",
        default="/orb_slam3/robot_pose_slam",
        help="PoseStamped topic from ORB-SLAM3",
    )
    parser.add_argument(
        "--slam-info-topic",
        default=SLAM_INFO_TOPIC,
        help="slam_info topic (tracking_frequency etc.)",
    )
    parser.add_argument(
        "--pose-freshness",
        type=float,
        default=POSE_FRESHNESS_S,
        help="Seconds a pose may be stale before tracking is considered lost",
    )
    parser.add_argument(
        "--flag-file",
        default="/tmp/terraslam_save_frames",
        help="Capture only while this file exists",
    )
    # ROS injects `__node:=...` style args; drop them before argparse.
    raw_args = [a for a in sys.argv[1:] if not a.startswith("__")]
    parsed = parser.parse_args(raw_args)

    if not os.path.isdir(parsed.procframe_dir):
        print(f"procframe dir does not exist: {parsed.procframe_dir}", file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = ProcFrameCapture(
        procframe_dir=parsed.procframe_dir,
        image_topic=parsed.image_topic,
        pose_topic=parsed.pose_topic,
        slam_info_topic=parsed.slam_info_topic,
        flag_file=parsed.flag_file,
        pose_freshness_s=parsed.pose_freshness,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"Shutting down: saved={node.saved} failed={node.failed}"
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
