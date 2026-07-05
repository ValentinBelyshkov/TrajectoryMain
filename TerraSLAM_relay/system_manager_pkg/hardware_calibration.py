"""
Hardware calibration — configures ArduPilot's EKF3 to accept ORB-SLAM
external-navigation ("vision") data instead of GPS, on the real flight
controller over MAVLink.

This wraps the parameter-setting logic used for GPS-denied SLAM-guided
missions (AHRS_EKF_TYPE, EK3_SRC1_*, VISO_*, FENCE_ENABLE, ...). It is
invoked on demand via POST /api/v1/hardware/calibrate, and runs the
blocking pymavlink calls in a worker thread so it doesn't block the
FastAPI event loop.
"""
import asyncio
import time
from typing import Any, Dict, List, Tuple

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except ImportError:
    mavutil = None
    PYMAVLINK_AVAILABLE = False


# (param_id, value, description)
EKF3_VISION_PARAMS: List[Tuple[str, float, str]] = [
    ("AHRS_EKF_TYPE", 3, "Use EKF3"),
    ("EK3_ENABLE", 1, "Enable EKF3"),
    ("EK3_SRC1_POSXY", 6, "Horizontal position source: ExternalNav"),
    ("EK3_SRC1_VELXY", 0, "Horizontal velocity source: none"),
    ("EK3_SRC1_POSZ", 1, "Vertical position source: Baro"),
    ("EK3_SRC1_VELZ", 0, "Vertical velocity source: none"),
    ("EK3_SRC1_YAW", 1, "Yaw source: compass"),
    ("EK3_SRC_OPTIONS", 0, "No source-fusion options"),
    ("GPS1_TYPE", 0, "Disable GPS1"),
    ("VISO_TYPE", 1, "Enable MAVLink vision odometry"),
    ("VISO_POS_X", 0.0, "Vision sensor offset X"),
    ("VISO_POS_Y", 0.0, "Vision sensor offset Y"),
    ("VISO_POS_Z", 0.0, "Vision sensor offset Z"),
    ("FENCE_ENABLE", 0, "Disable geofence (no GPS to check it against)"),
]


class HardwareCalibrationError(Exception):
    pass


class ArduPilotEkf3Calibrator:
    """Connects to a real (or SITL) flight controller and pushes the
    EKF3 + external-navigation parameter set required for GPS-denied,
    ORB-SLAM-guided flight."""

    def __init__(self, connection: str, connect_timeout: float = 10.0):
        if not PYMAVLINK_AVAILABLE:
            raise HardwareCalibrationError(
                "pymavlink is not installed — hardware calibration requires the "
                "Docker/ROS environment with pymavlink available."
            )
        self.connection_str = connection
        self.connect_timeout = connect_timeout
        self.master = None

    def connect(self):
        self.master = mavutil.mavlink_connection(self.connection_str)
        msg = self.master.wait_heartbeat(timeout=self.connect_timeout)
        if msg is None:
            raise HardwareCalibrationError(
                f"No heartbeat from flight controller at {self.connection_str} "
                f"within {self.connect_timeout}s"
            )
        self.sys = self.master.target_system
        self.comp = self.master.target_component

    def _set_param(self, param_id: str, value: float, retries: int = 3) -> Dict[str, Any]:
        param_id_bytes = param_id.encode("utf-8")[:16].ljust(16, b"\0")
        for attempt in range(retries):
            self.master.mav.param_set_send(
                self.sys, self.comp, param_id_bytes, float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            t_start = time.time()
            while time.time() - t_start < 5.0:
                msg = self.master.recv_match(type="PARAM_VALUE", blocking=False)
                if msg:
                    recv_id = msg.param_id
                    if isinstance(recv_id, bytes):
                        recv_id = recv_id.rstrip(b"\x00").decode("utf-8", errors="ignore")
                    else:
                        recv_id = str(recv_id).rstrip("\x00")
                    if recv_id == param_id:
                        recv_val = float(msg.param_value)
                        ok = abs(recv_val - float(value)) < 0.01
                        return {"param": param_id, "requested": value, "confirmed": recv_val, "ok": ok}
                time.sleep(0.05)
        return {"param": param_id, "requested": value, "confirmed": None, "ok": False}

    def calibrate_for_vision(self) -> Dict[str, Any]:
        """Runs the full EKF3 + external-nav parameter push. This is the
        hardware calibration routine — equivalent to configure_ekf3_for_vision()
        from the SLAM-guided-mission reference script, but reusable as a
        library call from the system manager API."""
        self.connect()
        results = []
        all_ok = True
        for param_id, value, description in EKF3_VISION_PARAMS:
            r = self._set_param(param_id, value)
            r["description"] = description
            results.append(r)
            if not r["ok"]:
                all_ok = False
        return {
            "success": all_ok,
            "connection": self.connection_str,
            "results": results,
            "note": "Reboot the flight controller for the new EKF3 source parameters to take effect.",
        }


async def run_hardware_calibration(connection: str, timeout: float = 15.0) -> Dict[str, Any]:
    """Async entry point used by the API layer. Runs the blocking MAVLink
    calibration in a worker thread."""
    loop = asyncio.get_event_loop()

    def _run():
        calibrator = ArduPilotEkf3Calibrator(connection, connect_timeout=timeout)
        return calibrator.calibrate_for_vision()

    return await loop.run_in_executor(None, _run)
