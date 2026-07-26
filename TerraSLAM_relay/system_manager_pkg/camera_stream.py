"""
Persistent rclpy subscriber for /camera/image_raw.

Runs a single long-lived ROS 2 node in a background thread and stores
every incoming frame as a compressed JPEG in memory (latest frame only).
The MJPEG streaming endpoint reads from this shared state without any
per-request ROS I/O or DDS discovery delay.
"""
import os
import threading
import time
from typing import Optional, Tuple

os.environ.setdefault("ROS_DOMAIN_ID", "0")

try:
    import rclpy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    DEPS_AVAILABLE = True
    _IMPORT_ERROR = ""
except ImportError as e:
    DEPS_AVAILABLE = False
    _IMPORT_ERROR = str(e)

CAMERA_TOPIC = "/camera/image_raw"
JPEG_QUALITY = 75  # 0-100; lower = smaller, faster network transfer

# ── shared state ─────────────────────────────────────────────────────────────
_lock = threading.Lock()
_latest_jpeg: Optional[bytes] = None
_last_frame_ts: Optional[float] = None
_frame_count: int = 0

_running = False
_thread: Optional[threading.Thread] = None
_start_error: str = ""


def get_latest_frame() -> Tuple[Optional[bytes], Optional[float]]:
    """Return (jpeg_bytes, unix_timestamp) of the latest frame, or (None, None)."""
    with _lock:
        return _latest_jpeg, _last_frame_ts


def frame_count() -> int:
    with _lock:
        return _frame_count


# ── internal ─────────────────────────────────────────────────────────────────
def _frame_cb(msg: "Image") -> None:  # type: ignore[name-defined]
    global _latest_jpeg, _last_frame_ts, _frame_count
    try:
        bridge = _bridge_holder[0]
        if bridge is None:
            return
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", cv_img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            with _lock:
                _latest_jpeg = buf.tobytes()
                _last_frame_ts = time.time()
                _frame_count += 1
    except Exception:
        pass


_bridge_holder: list = [None]


def _spin() -> None:
    global _start_error
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
        _bridge_holder[0] = CvBridge()
        node = rclpy.create_node("terraslam_camera_stream")
        node.create_subscription(Image, CAMERA_TOPIC, _frame_cb, 10)
        _start_error = ""
        while _running:
            rclpy.spin_once(node, timeout_sec=0.5)
        try:
            node.destroy_node()
        except Exception:
            pass
    except Exception as e:
        _start_error = str(e)


# ── public API ────────────────────────────────────────────────────────────────
def start() -> None:
    """Idempotent: safe to call repeatedly."""
    global _running, _thread, _start_error
    if not DEPS_AVAILABLE:
        _start_error = f"Camera/CV deps not available: {_IMPORT_ERROR}"
        return
    if _running and _thread is not None and _thread.is_alive():
        return
    _running = True
    _thread = threading.Thread(target=_spin, name="camera-stream", daemon=True)
    _thread.start()


def stop() -> None:
    global _running
    _running = False


def is_running() -> bool:
    return _running and _thread is not None and _thread.is_alive()
