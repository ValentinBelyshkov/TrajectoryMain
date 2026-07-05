"""
Fallback (failsafe) controller — decides what the vehicle should do when
SLAM pose tracking is lost or stale, and sends the corresponding flight
mode change to the flight controller over MAVLink.

Trigger conditions (checked by PoseMonitor):
  - the pose topic goes stale (no message within POSE_STALE_AFTER_SECONDS)
  - ORB-SLAM3 publishes the "tracking lost" sentinel pose
    (position.x == position.y == position.z == TRACKING_LOST_SENTINEL, i.e. -3.0)

The action taken is configurable via get/set (see routes_slam.py):
  - "POSHOLD" — hold current position/altitude
  - "RTL"     — return to launch
  - "LAND"    — land immediately
"""
import time
from typing import Any, Dict, Optional

from .config import DEFAULT_FALLBACK_CONNECTION, DEFAULT_FALLBACK_METHOD, VALID_FALLBACK_METHODS

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except ImportError:
    mavutil = None
    PYMAVLINK_AVAILABLE = False


class FallbackError(Exception):
    pass


# ArduCopter custom flight-mode numbers (APM:Copter mode table).
_ARDUCOPTER_MODE_NUMBERS = {
    "POSHOLD": 16,
    "RTL": 6,
    "LAND": 9,
}


class FallbackController:
    """Holds the currently configured fallback method/connection and can
    execute it against a real (or SITL) flight controller over MAVLink."""

    def __init__(self, method: str = DEFAULT_FALLBACK_METHOD, connection: str = DEFAULT_FALLBACK_CONNECTION):
        self._method = DEFAULT_FALLBACK_METHOD
        self.set_method(method)
        self.connection_str = connection
        self._last_trigger: Optional[Dict[str, Any]] = None

    # --- getter/setter for the fallback method ---
    def get_method(self) -> str:
        return self._method

    def set_method(self, method: str) -> str:
        method = (method or "").strip().upper()
        if method not in VALID_FALLBACK_METHODS:
            raise FallbackError(f"method must be one of {VALID_FALLBACK_METHODS}, got {method!r}")
        self._method = method
        return self._method

    # --- getter/setter for the MAVLink connection string ---
    def get_connection(self) -> str:
        return self.connection_str

    def set_connection(self, connection: str) -> str:
        if not connection:
            raise FallbackError("connection must be a non-empty MAVLink connection string")
        self.connection_str = connection
        return self.connection_str

    def last_trigger(self) -> Optional[Dict[str, Any]]:
        return self._last_trigger

    def _send_mode_change(self, method: str) -> Dict[str, Any]:
        if not PYMAVLINK_AVAILABLE:
            return {
                "success": False,
                "method": method,
                "error": "pymavlink is not installed — fallback requires the Docker/ROS environment.",
            }
        try:
            master = mavutil.mavlink_connection(self.connection_str)
            hb = master.wait_heartbeat(timeout=10.0)
            if hb is None:
                return {
                    "success": False,
                    "method": method,
                    "error": f"No heartbeat from flight controller at {self.connection_str}",
                }
            mode_id = _ARDUCOPTER_MODE_NUMBERS[method]
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            return {"success": True, "method": method, "mode_id": mode_id, "connection": self.connection_str}
        except Exception as e:
            return {"success": False, "method": method, "error": str(e)}

    def trigger(self, reason: str) -> Dict[str, Any]:
        """Executes the currently configured fallback action. Called by the
        pose monitor when tracking is lost or the pose topic goes stale."""
        result = self._send_mode_change(self._method)
        result["reason"] = reason
        result["timestamp"] = time.time()
        self._last_trigger = result
        return result


fallback_controller = FallbackController()
