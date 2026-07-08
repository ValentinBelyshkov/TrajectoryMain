"""
Persistent rclpy subscriber for /camera_pose and /orb_slam3/tracking_state.

Why this exists: the previous implementation polled by spawning a fresh
`ros2 topic echo --once` subprocess on every tick. Each subprocess has to
start a new Python interpreter, call rclpy.init(), and wait for DDS
discovery to find the publisher before it can receive a single message —
that setup cost alone is typically 0.5-3s and varies with system load,
which is exactly the jitter seen in the WS panel. It also only ever
captures the state at each poll instant and can miss everything published
in between.

This module instead starts ONE long-lived ROS2 node, in a background
thread, that stays subscribed to both topics for the lifetime of the
process. Messages update in-memory state the instant they arrive (sub-ms),
so anything reading that state (pose_monitor.py) gets a plain, fast, non
-blocking dict read — no per-check ROS2 I/O or discovery cost at all.
"""
import os
import threading
import time
from typing import Any, Dict, Optional, Tuple

from .config import POSE_TOPIC, TRACKING_STATE_TOPIC

os.environ.setdefault("ROS_DOMAIN_ID", "0")

try:
    import rclpy
    from geometry_msgs.msg import Pose
    from std_msgs.msg import Int8
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False


class _PoseSubscriberState:
    def __init__(self):
        self._lock = threading.Lock()
        self.last_position: Optional[Tuple[float, float, float]] = None
        self.last_payload: str = ""
        self.last_seen: Optional[float] = None
        self.tracking_state: Optional[int] = None
        self.tracking_state_last_seen: Optional[float] = None

    def set_pose(self, position: Tuple[float, float, float], payload: str) -> None:
        with self._lock:
            self.last_position = position
            self.last_payload = payload
            self.last_seen = time.time()

    def set_tracking_state(self, value: int) -> None:
        with self._lock:
            self.tracking_state = value
            self.tracking_state_last_seen = time.time()

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "last_position": self.last_position,
                "last_payload": self.last_payload,
                "last_seen": self.last_seen,
                "tracking_state": self.tracking_state,
                "tracking_state_last_seen": self.tracking_state_last_seen,
            }


state = _PoseSubscriberState()

_node = None
_thread: Optional[threading.Thread] = None
_running = False
_start_error: str = ""


def _pose_cb(msg) -> None:
    p = msg.position
    o = msg.orientation
    payload = (
        f"position:\n  x: {p.x}\n  y: {p.y}\n  z: {p.z}\n"
        f"orientation:\n  x: {o.x}\n  y: {o.y}\n  z: {o.z}\n  w: {o.w}"
    )
    state.set_pose((float(p.x), float(p.y), float(p.z)), payload)


def _tracking_state_cb(msg) -> None:
    state.set_tracking_state(int(msg.data))


def _spin() -> None:
    global _node, _start_error
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
        _node = rclpy.create_node("terraslam_pose_monitor_subscriber")
        _node.create_subscription(Pose, POSE_TOPIC, _pose_cb, 10)
        _node.create_subscription(Int8, TRACKING_STATE_TOPIC, _tracking_state_cb, 10)
        _start_error = ""
        while _running:
            rclpy.spin_once(_node, timeout_sec=0.5)
    except Exception as e:
        _start_error = str(e)
    finally:
        if _node is not None:
            try:
                _node.destroy_node()
            except Exception:
                pass
            _node = None
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def start() -> None:
    """Idempotent: safe to call repeatedly (e.g. on every PoseMonitor.start())."""
    global _running, _thread, _start_error
    if not RCLPY_AVAILABLE:
        _start_error = "rclpy not available — run inside the ROS2 environment on the host"
        return
    if _running and _thread is not None and _thread.is_alive():
        return
    _running = True
    _thread = threading.Thread(target=_spin, name="pose-subscriber", daemon=True)
    _thread.start()


def stop() -> None:
    global _running
    _running = False


def is_running() -> bool:
    return _running and _thread is not None and _thread.is_alive()


def get_status_extra() -> Dict[str, Any]:
    return {
        "rclpy_available": RCLPY_AVAILABLE,
        "subscriber_running": is_running(),
        "start_error": _start_error,
    }
