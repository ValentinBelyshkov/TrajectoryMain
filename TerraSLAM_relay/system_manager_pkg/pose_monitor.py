"""
Pose liveness check — verifies that ORB-SLAM3 is actually publishing pose
messages on the ROS2 topic, rather than just reporting the process as
"running".

Pose data is fed by a single persistent rclpy subscriber node
(ros_pose_subscriber.py) that stays subscribed to /camera_pose and
/orb_slam3/tracking_state for the process lifetime. A background asyncio
loop here just reads that in-memory state at a fixed interval (cheap,
non-blocking) and evaluates the fallback condition — it no longer spawns
`ros2 topic echo` subprocesses, which used to add 0.5-3s of process start
+ DDS discovery jitter to every single check.

Also detects the ORB-SLAM3 "tracking lost" sentinel pose
(position.x = position.y = position.z = -3.0). When the pose topic goes
stale OR the tracking-lost sentinel is seen, status becomes "fallback" and
the configured fallback action (PosHold/RTL/Land) is sent to the flight
controller via fallback_controller.
"""
import asyncio
import time
from typing import Any, Dict, Optional, Tuple

from . import ros_pose_subscriber
from .config import (
    POSE_POLL_INTERVAL_SECONDS,
    POSE_STALE_AFTER_SECONDS,
    POSE_TOPIC,
    TRACKING_LOST_EPSILON,
    TRACKING_LOST_SENTINEL,
    TRACKING_STATE_LOST_VALUES,
    TRACKING_STATE_TOPIC,
)
from .fallback_controller import fallback_controller


def _is_tracking_lost(position: Optional[Tuple[float, float, float]]) -> bool:
    if position is None:
        return False
    x, y, z = position
    return (
        abs(x - TRACKING_LOST_SENTINEL) < TRACKING_LOST_EPSILON
        and abs(y - TRACKING_LOST_SENTINEL) < TRACKING_LOST_EPSILON
        and abs(z - TRACKING_LOST_SENTINEL) < TRACKING_LOST_EPSILON
    )


class PoseMonitor:
    """Tracks whether /camera_pose is actively publishing, and cross-checks
    the ORB-SLAM3 tracking state on /orb_slam3/tracking_state. Reads from
    the persistent rclpy subscriber in ros_pose_subscriber.py instead of
    polling via subprocess."""

    def __init__(self, topic: str = POSE_TOPIC, poll_interval: float = POSE_POLL_INTERVAL_SECONDS,
                 stale_after: float = POSE_STALE_AFTER_SECONDS, tracking_state_topic: str = TRACKING_STATE_TOPIC):
        self.topic = topic
        self.tracking_state_topic = tracking_state_topic
        self.poll_interval = poll_interval
        self.stale_after = stale_after
        self._last_seen: Optional[float] = None
        self._last_payload: str = ""
        self._last_error: str = ""
        self._last_position: Optional[Tuple[float, float, float]] = None
        self._tracking_state: Optional[int] = None
        self._tracking_state_last_seen: Optional[float] = None
        self._tracking_state_error: str = ""
        self._tracking_lost = False
        self._fallback_active = False
        self._task: Optional[asyncio.Task] = None
        self._running = False

    def start(self):
        ros_pose_subscriber.start()
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
        ros_pose_subscriber.stop()

    def _apply_snapshot(self) -> None:
        """Pulls the latest values from the persistent subscriber's shared
        state. Cheap in-memory read, no ROS2 I/O."""
        snap = ros_pose_subscriber.state.snapshot()

        if snap["last_seen"] is not None:
            self._last_seen = snap["last_seen"]
            self._last_payload = snap["last_payload"]
            self._last_position = snap["last_position"]
            self._tracking_lost = _is_tracking_lost(self._last_position)
            self._last_error = ""
        elif not ros_pose_subscriber.is_running():
            extra = ros_pose_subscriber.get_status_extra()
            self._last_error = extra["start_error"] or "pose subscriber not running"

        if snap["tracking_state_last_seen"] is not None:
            self._tracking_state_last_seen = snap["tracking_state_last_seen"]
            self._tracking_state = snap["tracking_state"]
            self._tracking_state_error = ""
        elif not ros_pose_subscriber.is_running():
            extra = ros_pose_subscriber.get_status_extra()
            self._tracking_state_error = extra["start_error"] or "pose subscriber not running"

    def _should_trigger_fallback(self) -> Optional[str]:
        """Decides whether a fallback should fire (pose stale after having
        been seen before, the tracking-lost sentinel observed, or the
        tracking-state topic reporting RECENTLY_LOST/LOST), on a rising
        edge only (once per loss episode). Resets once a normal, non-lost
        pose returns. Returns the trigger reason, or None if no trigger
        is needed. Does not perform any blocking I/O."""
        now = time.time()
        age = (now - self._last_seen) if self._last_seen else None
        stale = self._last_seen is not None and age is not None and age > self.stale_after
        tracking_state_lost = self._tracking_state in TRACKING_STATE_LOST_VALUES
        should_fallback = stale or self._tracking_lost or tracking_state_lost

        if should_fallback and not self._fallback_active:
            self._fallback_active = True
            if self._tracking_lost:
                return "tracking_lost"
            if tracking_state_lost:
                return "tracking_state_lost"
            return "pose_stale"
        if not should_fallback:
            self._fallback_active = False
        return None

    async def _poll_loop(self):
        loop = asyncio.get_event_loop()
        while self._running:
            self._apply_snapshot()
            reason = self._should_trigger_fallback()
            if reason is not None:
                await loop.run_in_executor(None, fallback_controller.trigger, reason)
            await asyncio.sleep(self.poll_interval)

    def check_once(self, timeout: float = 3.0) -> Dict[str, Any]:
        """On-demand check (used by the /pose_status endpoint). The
        persistent subscriber is always streaming in the background, so
        this is just an immediate read of its current state — `timeout`
        is kept for API compatibility but is no longer a blocking wait."""
        ros_pose_subscriber.start()
        self._apply_snapshot()
        reason = self._should_trigger_fallback()
        if reason is not None:
            fallback_controller.trigger(reason)
        return self.status()

    def status(self) -> Dict[str, Any]:
        now = time.time()
        age = (now - self._last_seen) if self._last_seen else None
        alive = age is not None and age <= self.stale_after
        state = "fallback" if self._fallback_active else ("alive" if alive else "unknown")

        tracking_age = (now - self._tracking_state_last_seen) if self._tracking_state_last_seen else None
        tracking_state_lost = self._tracking_state in TRACKING_STATE_LOST_VALUES

        return {
            "topic": self.topic,
            "alive": alive,
            "state": state,
            "last_seen": self._last_seen,
            "age_seconds": round(age, 3) if age is not None else None,
            "stale_after_seconds": self.stale_after,
            "last_position": self._last_position,
            "tracking_lost": self._tracking_lost,
            "tracking_state": {
                "topic": self.tracking_state_topic,
                "value": self._tracking_state,
                "lost": tracking_state_lost,
                "last_seen": self._tracking_state_last_seen,
                "age_seconds": round(tracking_age, 3) if tracking_age is not None else None,
                "last_error": self._tracking_state_error,
            },
            "fallback_active": self._fallback_active,
            "last_fallback": fallback_controller.last_trigger(),
            "fallback_method": fallback_controller.get_method(),
            "last_payload": self._last_payload[:2000] if self._last_payload else "",
            "last_error": self._last_error,
            "subscriber": ros_pose_subscriber.get_status_extra(),
        }


pose_monitor = PoseMonitor()
