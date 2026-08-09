"""
Persistent rclpy subscriber for /orb_slam3/robot_pose_slam (geometry_msgs/PoseStamped)
and /orb_slam3/slam_info (slam_msgs/SlamInfo).

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

from .config import POSE_TOPIC, SLAM_INFO_TOPIC

os.environ.setdefault("ROS_DOMAIN_ID", "0")

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from slam_msgs.msg import SlamInfo
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
        self.tracking_frequency: float = 0.0
        self.num_keyframes: int = 0

    def set_pose(self, position: Tuple[float, float, float], payload: str) -> None:
        with self._lock:
            self.last_position = position
            self.last_payload = payload
            self.last_seen = time.time()

    def set_slam_info(self, tracking_frequency: float, num_keyframes: int) -> None:
        with self._lock:
            self.tracking_frequency = tracking_frequency
            self.num_keyframes = num_keyframes
            self.tracking_state_last_seen = time.time()
            # Derive a coarse legacy state value: 2 = OK, 3 = recently lost.
            if tracking_frequency > 0.1:
                self.tracking_state = 2
            elif num_keyframes > 0:
                self.tracking_state = 3
            else:
                self.tracking_state = 1

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "last_position": self.last_position,
                "last_payload": self.last_payload,
                "last_seen": self.last_seen,
                "tracking_state": self.tracking_state,
                "tracking_state_last_seen": self.tracking_state_last_seen,
                "tracking_frequency": self.tracking_frequency,
                "num_keyframes": self.num_keyframes,
            }


state = _PoseSubscriberState()

_node = None
_thread: Optional[threading.Thread] = None
_running = False
_start_error: str = ""


def _pose_cb(msg) -> None:
    p = msg.pose.position
    o = msg.pose.orientation
    payload = (
        f"position:\n  x: {p.x}\n  y: {p.y}\n  z: {p.z}\n"
        f"orientation:\n  x: {o.x}\n  y: {o.y}\n  z: {o.z}\n  w: {o.w}"
    )
    state.set_pose((float(p.x), float(p.y), float(p.z)), payload)


def _slam_info_cb(msg) -> None:
    state.set_slam_info(float(msg.tracking_frequency), int(msg.num_keyframes_in_current_map))


def _spin() -> None:
    global _node, _start_error
    executor = None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
        _node = rclpy.create_node("terraslam_pose_monitor_subscriber")
        # Use an explicit executor (not the rclpy.spin_once helper which creates
        # a temporary SingleThreadedExecutor on every call — that leaks the
        # _sigint_gc attribute and causes "AttributeError: … no attribute
        # '_sigint_gc'" noise in the logs, and is unreliable under load).
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(_node)
        _node.create_subscription(PoseStamped, POSE_TOPIC, _pose_cb, 10)
        _node.create_subscription(SlamInfo, SLAM_INFO_TOPIC, _slam_info_cb, 10)
        _start_error = ""
        while _running:
            executor.spin_once(timeout_sec=0.5)
    except Exception as e:
        _start_error = str(e)
    finally:
        if executor is not None:
            try:
                if _node is not None:
                    executor.remove_node(_node)
                executor.shutdown()
            except Exception:
                pass
        if _node is not None:
            try:
                _node.destroy_node()
            except Exception:
                pass
            _node = None
        # NOTE: do NOT call rclpy.shutdown() here — this is one of several
        # modules sharing the same rclpy context (camera_stream, camera_snapshot).
        # Shutting it down would silently break them all.


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

