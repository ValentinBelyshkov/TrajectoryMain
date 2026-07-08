"""
On-demand single-frame camera snapshot.

Grabs one message from /camera/image_raw (sensor_msgs/Image) via a
short-lived rclpy subscription, saves it as an image file at a
user-specified path, and writes a sibling `<same-basename>.txt` file with
the current /camera_pose reading — reused directly from the already
-running persistent pose subscriber in ros_pose_subscriber.py, so no extra
pose topic I/O is needed here.
"""
import os
import time
from typing import Any, Dict, Optional

from . import ros_pose_subscriber
from .config import POSE_TOPIC

try:
    import rclpy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    CAMERA_DEPS_AVAILABLE = True
    _IMPORT_ERROR = ""
except ImportError as e:
    CAMERA_DEPS_AVAILABLE = False
    _IMPORT_ERROR = str(e)

CAMERA_TOPIC = "/camera/image_raw"


def _grab_one_frame(topic: str, timeout: float):
    """Blocking: subscribes to `topic` on the shared rclpy context (already
    initialized by the persistent pose subscriber) and waits for a single
    Image message. Returns (cv_image_bgr, error).

    Uses an explicit SingleThreadedExecutor (instead of the bare
    rclpy.spin_once(node, ...) helper, which internally allocates and
    tears down its own temporary executor on every call) so no executor
    object is left to be garbage-collected after the fact — that stray
    cleanup is what produces the harmless-but-noisy
    "'SingleThreadedExecutor' object has no attribute '_sigint_gc'"
    message in the logs."""
    if not rclpy.ok():
        rclpy.init(args=None)

    bridge = CvBridge()
    node = rclpy.create_node("terraslam_camera_snapshot")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    holder: Dict[str, Any] = {"msg": None}

    def _cb(msg):
        holder["msg"] = msg

    sub = node.create_subscription(Image, topic, _cb, 1)
    try:
        # Give discovery a brief moment before deciding no one is
        # publishing at all — much more useful than a generic timeout.
        discovery_deadline = time.monotonic() + min(1.5, timeout)
        while node.count_publishers(topic) == 0 and time.monotonic() < discovery_deadline:
            executor.spin_once(timeout_sec=0.1)
        if node.count_publishers(topic) == 0:
            return None, (
                f"No publisher found on {topic} — is the camera/image "
                f"publisher component (publisher_folder / publisher_realsense) running?"
            )

        deadline = time.monotonic() + timeout
        while holder["msg"] is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.2)
        if holder["msg"] is None:
            return None, f"Publisher found on {topic} but no message received within {timeout}s"
        cv_image = bridge.imgmsg_to_cv2(holder["msg"], desired_encoding="bgr8")
        return cv_image, None
    except Exception as e:
        return None, str(e)
    finally:
        executor.remove_node(node)
        node.destroy_subscription(sub)
        node.destroy_node()
        executor.shutdown()


def save_snapshot(image_path: str, timeout: float = 5.0) -> Dict[str, Any]:
    """Saves one frame from /camera/image_raw to `image_path`, and the
    current /camera_pose reading to a sibling `<basename>.txt`."""
    if not CAMERA_DEPS_AVAILABLE:
        return {"success": False, "error": f"camera dependencies not available: {_IMPORT_ERROR}"}

    directory = os.path.dirname(image_path) or "."
    if not os.path.isdir(directory):
        return {"success": False, "error": f"directory does not exist: {directory}"}

    cv_image, error = _grab_one_frame(CAMERA_TOPIC, timeout)
    if error is not None:
        return {"success": False, "error": error}

    if not cv2.imwrite(image_path, cv_image):
        return {"success": False, "error": f"cv2.imwrite failed for {image_path}"}

    base, _ext = os.path.splitext(image_path)
    txt_path = base + ".txt"
    snap = ros_pose_subscriber.state.snapshot()
    last_seen: Optional[float] = snap["last_seen"]
    pose_age = (time.time() - last_seen) if last_seen else None
    lines = [
        f"topic: {POSE_TOPIC}",
        f"last_seen_unix: {last_seen}",
        f"age_seconds: {round(pose_age, 3) if pose_age is not None else None}",
        "",
        snap["last_payload"] or "no pose received yet",
    ]
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    return {
        "success": True,
        "image_path": image_path,
        "pose_path": txt_path,
        "pose_age_seconds": round(pose_age, 3) if pose_age is not None else None,
    }

