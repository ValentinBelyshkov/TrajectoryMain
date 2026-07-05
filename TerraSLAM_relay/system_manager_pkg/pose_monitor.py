"""
Pose liveness check — verifies that ORB-SLAM3 is actually publishing pose
messages on the ROS2 topic, rather than just reporting the process as
"running". A background asyncio task polls `ros2 topic echo --once` at a
fixed interval and records when the last pose was seen.

Also detects the ORB-SLAM3 "tracking lost" sentinel pose
(position.x = position.y = position.z = -3.0). When the pose topic goes
stale OR the tracking-lost sentinel is seen, status becomes "fallback" and
the configured fallback action (PosHold/RTL/Land) is sent to the flight
controller via fallback_controller.
"""
import asyncio
import os
import subprocess
import time
from typing import Any, Dict, Optional, Tuple

try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    yaml = None
    YAML_AVAILABLE = False

from .config import (
    POSE_POLL_INTERVAL_SECONDS,
    POSE_STALE_AFTER_SECONDS,
    POSE_TOPIC,
    TRACKING_LOST_EPSILON,
    TRACKING_LOST_SENTINEL,
)
from .fallback_controller import fallback_controller


def _extract_position(stdout: str) -> Optional[Tuple[float, float, float]]:
    """Best-effort extraction of an (x, y, z) position from a `ros2 topic
    echo` YAML dump, regardless of exact message type (PoseStamped,
    Odometry, etc.) — searches recursively for a dict with numeric x/y/z."""
    if not YAML_AVAILABLE:
        return None
    try:
        data = yaml.safe_load(stdout)
    except Exception:
        return None

    def search(node):
        if isinstance(node, dict):
            if all(k in node for k in ("x", "y", "z")) and all(
                isinstance(node[k], (int, float)) for k in ("x", "y", "z")
            ):
                return (float(node["x"]), float(node["y"]), float(node["z"]))
            for value in node.values():
                found = search(value)
                if found is not None:
                    return found
        elif isinstance(node, list):
            for item in node:
                found = search(item)
                if found is not None:
                    return found
        return None

    return search(data)


def _is_tracking_lost(position: Optional[Tuple[float, float, float]]) -> bool:
    if position is None:
        return False
    x, y, z = position
    return (
        abs(x - TRACKING_LOST_SENTINEL) < TRACKING_LOST_EPSILON
        and abs(y - TRACKING_LOST_SENTINEL) < TRACKING_LOST_EPSILON
        and abs(z - TRACKING_LOST_SENTINEL) < TRACKING_LOST_EPSILON
    )


def _ros2_topic_echo_once(topic: str, timeout: float) -> Dict[str, Any]:
    cmd = ["ros2", "topic", "echo", "--once", topic]
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout,
            env={**os.environ, "ROS_DOMAIN_ID": "0"},
        )
        return {
            "stdout": proc.stdout.strip(),
            "stderr": proc.stderr.strip(),
            "returncode": proc.returncode,
        }
    except FileNotFoundError:
        return {"stdout": "", "stderr": "ros2 not found — run inside the Docker container", "returncode": 127}
    except subprocess.TimeoutExpired:
        return {"stdout": "", "stderr": f"Timeout after {timeout}s", "returncode": 1}
    except Exception as e:
        return {"stdout": "", "stderr": str(e), "returncode": 1}


class PoseMonitor:
    """Tracks whether /orb_slam3/camera_pose (or equivalent) is actively publishing."""

    def __init__(self, topic: str = POSE_TOPIC, poll_interval: float = POSE_POLL_INTERVAL_SECONDS,
                 stale_after: float = POSE_STALE_AFTER_SECONDS):
        self.topic = topic
        self.poll_interval = poll_interval
        self.stale_after = stale_after
        self._last_seen: Optional[float] = None
        self._last_payload: str = ""
        self._last_error: str = ""
        self._last_position: Optional[Tuple[float, float, float]] = None
        self._tracking_lost = False
        self._fallback_active = False
        self._task: Optional[asyncio.Task] = None
        self._running = False

    def start(self):
        if self._task is None or self._task.done():
            self._running = True
            self._task = asyncio.create_task(self._poll_loop())

    async def stop(self):
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):
                pass
            self._task = None

    def _handle_result(self, result: Dict[str, Any]) -> None:
        """Updates internal state from one echo attempt. Does NOT trigger
        the fallback action — that's decided by _evaluate_fallback so it
        applies identically whether called from the poll loop or check_once."""
        if result["returncode"] == 0 and result["stdout"]:
            self._last_seen = time.time()
            self._last_payload = result["stdout"]
            self._last_error = ""
            self._last_position = _extract_position(result["stdout"])
            self._tracking_lost = _is_tracking_lost(self._last_position)
        else:
            self._last_error = result["stderr"]

    def _should_trigger_fallback(self) -> Optional[str]:
        """Decides whether a fallback should fire (pose stale after having
        been seen before, or tracking-lost sentinel observed), on a rising
        edge only (once per loss episode). Resets once a normal, non-lost
        pose returns. Returns the trigger reason, or None if no trigger
        is needed. Does not perform any blocking I/O."""
        now = time.time()
        age = (now - self._last_seen) if self._last_seen else None
        stale = self._last_seen is not None and age is not None and age > self.stale_after
        should_fallback = stale or self._tracking_lost

        if should_fallback and not self._fallback_active:
            self._fallback_active = True
            return "tracking_lost" if self._tracking_lost else "pose_stale"
        if not should_fallback:
            self._fallback_active = False
        return None

    async def _poll_loop(self):
        loop = asyncio.get_event_loop()
        while self._running:
            result = await loop.run_in_executor(
                None, _ros2_topic_echo_once, self.topic, max(self.poll_interval, 1.0)
            )
            self._handle_result(result)
            reason = self._should_trigger_fallback()
            if reason is not None:
                await loop.run_in_executor(None, fallback_controller.trigger, reason)
            await asyncio.sleep(self.poll_interval)

    def check_once(self, timeout: float = 3.0) -> Dict[str, Any]:
        """Synchronous, on-demand check (used by the /pose_status endpoint)."""
        result = _ros2_topic_echo_once(self.topic, timeout)
        self._handle_result(result)
        reason = self._should_trigger_fallback()
        if reason is not None:
            fallback_controller.trigger(reason)
        return self.status()

    def status(self) -> Dict[str, Any]:
        now = time.time()
        age = (now - self._last_seen) if self._last_seen else None
        alive = age is not None and age <= self.stale_after
        state = "fallback" if self._fallback_active else ("alive" if alive else "unknown")
        return {
            "topic": self.topic,
            "alive": alive,
            "state": state,
            "last_seen": self._last_seen,
            "age_seconds": round(age, 3) if age is not None else None,
            "stale_after_seconds": self.stale_after,
            "last_position": self._last_position,
            "tracking_lost": self._tracking_lost,
            "fallback_active": self._fallback_active,
            "last_fallback": fallback_controller.last_trigger(),
            "fallback_method": fallback_controller.get_method(),
            "last_payload": self._last_payload[:2000] if self._last_payload else "",
            "last_error": self._last_error,
        }


pose_monitor = PoseMonitor()
