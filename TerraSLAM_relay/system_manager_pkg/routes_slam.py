"""
SLAM run/mode/status/command routes, plus the new pose-liveness check and
the hardware EKF3 calibration trigger.
"""
import asyncio
import glob
import json
import os
import time

from fastapi import APIRouter, HTTPException

from .config import (
    FallbackConnectionReq,
    FallbackMethodReq,
    HardwareCalibrateReq,
    ModeReq,
    RunReq,
    ValueReq,
)
from .fallback_controller import FallbackError, fallback_controller
from .hardware_calibration import HardwareCalibrationError, run_hardware_calibration
from .pose_monitor import pose_monitor
from .process_manager import manager, update_yaml
from .ros_utils import ros2_run

router = APIRouter()

# Глобальная переменная для хранения последнего статуса SLAM
_slam_status = {
    "slam_state": -1,
    "slam_state_name": "UNKNOWN",
    "drone_mode": "LOCALIZATION_ONLY",
    "initialized": False,
    "timestamp": 0,
}


@router.post("/slam/run")
async def slam_run(req: RunReq):
    frames_dir = f"/home/orb/Database/projects/{req.project_id}/frames"
    if req.mode == "folder" and not os.path.isdir(frames_dir):
        raise HTTPException(status_code=400, detail=f"Frames dir missing: {frames_dir}")

    results = []
    yaml_path = "/home/orb/Database/real.yaml"
    save_path = f"/home/orb/Database/projects/{req.project_id}/calibrations/map"

    try:
        update_yaml(yaml_path, save_path)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"YAML update failed: {e}")

    publisher_name = f"publisher_{req.mode}"
    try:
        if req.mode == "folder":
            out = await manager.start("publisher_folder", extra={"path": frames_dir})
        else:
            out = await manager.start("publisher_realsense")
        results.append(f"publisher: {out}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Publisher failed: {e}")

    await asyncio.sleep(2)

    try:
        out = await manager.start("slam")
        results.append(f"slam: {out}")
    except Exception as e:
        try:
            await manager.stop(publisher_name)
        except Exception:
            pass
        raise HTTPException(status_code=500, detail=f"SLAM failed: {e}")

    await asyncio.sleep(req.duration)

    try:
        out = await manager.stop(publisher_name)
        results.append(f"publisher stop: {out}")
    except Exception as e:
        results.append(f"publisher stop error: {e}")

    await asyncio.sleep(2)

    try:
        out = await manager.stop("slam")
        results.append(f"slam stop: {out}")
    except Exception as e:
        results.append(f"slam stop error: {e}")

    calib_dir = f"/home/orb/Database/projects/{req.project_id}/calibrations"
    expected_osa = os.path.join(calib_dir, "map.osa")
    osa_found = None

    if os.path.exists(expected_osa):
        osa_found = expected_osa
    else:
        try:
            recent = sorted(
                glob.glob(os.path.join(calib_dir, "*.osa")),
                key=os.path.getmtime,
                reverse=True,
            )
            if recent:
                osa_found = recent[0]
        except Exception:
            pass

    return {
        "success": True,
        "results": results,
        "osa_file": osa_found,
        "project_id": req.project_id,
    }


@router.get("/api/v1/slam/status")
def get_slam_status():
    """Получить текущий статус SLAM (который приходит из ROS2 по /slam/mode_status)"""
    global _slam_status

    status_file = "/tmp/terraslam_slam_status"
    if os.path.exists(status_file):
        try:
            with open(status_file, "r") as f:
                content = f.read().strip()
            if content:
                parsed = json.loads(content)
                _slam_status.update(parsed)
                return {"success": True, "status": _slam_status, "manager_time": time.time()}
        except Exception as e:
            print(f"[get_slam_status] Error reading status file: {e}")

    stdout, stderr, rc = ros2_run(["topic", "echo", "--once", "/slam/mode_status"], timeout=2)
    if rc == 0 and stdout:
        try:
            content = stdout.strip()
            if content.startswith("data:"):
                data_str = content[5:].strip()
                if (data_str.startswith("'") and data_str.endswith("'")) or (data_str.startswith('"') and data_str.endswith('"')):
                    data_str = data_str[1:-1]
                data_str = data_str.replace("\\'", "'").replace('\\"', '"')
                parsed = json.loads(data_str)
                _slam_status.update(parsed)
        except Exception as e:
            print(f"[get_slam_status] Error parsing: {e}. Raw stdout: {stdout}")

    return {"success": True, "status": _slam_status, "manager_time": time.time()}


@router.post("/api/v1/slam/mode")
async def set_slam_mode(req: ModeReq):
    """
    Переключить режим SLAM.
    mode: "LOCALIZATION_ONLY" (команда 3) или "SLAM_MAPPING" (команда 4)
    """
    valid_modes = ["LOCALIZATION_ONLY", "SLAM_MAPPING"]
    if req.mode not in valid_modes:
        raise HTTPException(400, detail=f"Mode must be one of: {valid_modes}")

    try:
        mode_file = "/tmp/terraslam_slam_mode"
        with open(mode_file, "w") as f:
            f.write(req.mode)

        return {
            "success": True,
            "mode": req.mode,
            "message": f"Mode command written to {mode_file}. slam_mode_manager will process it.",
        }
    except Exception as e:
        raise HTTPException(500, detail=f"Failed to set mode: {e}")


@router.post("/api/v1/slam/command")
async def slam_command(req: ValueReq):
    """
    Прямые команды SLAM:
    - "reset" (команда 0)
    - "save_map" (команда 1) — требует filepath в req.value
    - "load_map" (команда 2) — требует filepath в req.value
    """
    if not req.value:
        raise HTTPException(400, detail="value required: reset | save_map | save_map|/path | load_map|/path")

    parts = req.value.split("|", 1)
    cmd = parts[0].strip().lower()
    filepath = parts[1].strip() if len(parts) > 1 else "/home/orb/Database/map.osa"

    valid = ["reset", "save_map", "load_map"]
    if cmd not in valid:
        raise HTTPException(400, detail=f"Command must be one of: {valid}")

    try:
        cmd_file = "/tmp/terraslam_slam_cmd"
        with open(cmd_file, "w") as f:
            f.write(f"{cmd}\n")

        if cmd in ["save_map", "load_map"]:
            path_file = "/tmp/terraslam_slam_path"
            with open(path_file, "w") as f:
                f.write(filepath)

        return {
            "success": True,
            "command": cmd,
            "filepath": filepath if cmd != "reset" else None,
            "message": f"Command '{cmd}' written to {cmd_file}",
        }
    except Exception as e:
        raise HTTPException(500, detail=f"Failed to write command: {e}")


# ========== POSE LIVENESS CHECK ==========

@router.get("/api/v1/slam/pose_status")
def get_pose_status():
    """Checks whether the SLAM pose topic is actively being published
    (not just that the slam process is running)."""
    return {"success": True, "pose": pose_monitor.status()}


@router.post("/api/v1/slam/pose_check")
def force_pose_check(timeout: float = 3.0):
    """Forces a synchronous check right now instead of waiting for the
    background poller's next cycle."""
    return {"success": True, "pose": pose_monitor.check_once(timeout=timeout)}


# ========== HARDWARE CALIBRATION ==========

@router.post("/api/v1/hardware/calibrate")
async def hardware_calibrate(req: HardwareCalibrateReq):
    """Calls the EKF3/ExternalNav calibration routine on the real flight
    controller over MAVLink (GPS off, vision-based localization on)."""
    try:
        result = await run_hardware_calibration(req.connection, timeout=req.timeout)
        return result
    except HardwareCalibrationError as e:
        raise HTTPException(status_code=503, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Calibration failed: {e}")


# ========== FALLBACK (FAILSAFE) ==========
# If the pose topic stops publishing, or ORB-SLAM3 reports tracking lost
# (position.x = position.y = position.z = -3.0), the system automatically
# sends the configured fallback command (PosHold/RTL/Land) to the flight
# controller. The method is configurable at runtime via get/set below.

@router.get("/api/v1/slam/fallback_method")
def get_fallback_method():
    """Returns the currently configured fallback action and MAVLink
    connection used when pose tracking is lost or the pose topic goes stale."""
    return {
        "success": True,
        "method": fallback_controller.get_method(),
        "connection": fallback_controller.get_connection(),
        "last_trigger": fallback_controller.last_trigger(),
    }


@router.post("/api/v1/slam/fallback_method")
def set_fallback_method(req: FallbackMethodReq):
    """Sets the fallback action to take when pose tracking is lost or the
    pose topic goes stale: POSHOLD | RTL | LAND."""
    try:
        method = fallback_controller.set_method(req.method)
        return {"success": True, "method": method}
    except FallbackError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/api/v1/slam/fallback_connection")
def set_fallback_connection(req: FallbackConnectionReq):
    """Sets the MAVLink connection string used to send the fallback command."""
    try:
        connection = fallback_controller.set_connection(req.connection)
        return {"success": True, "connection": connection}
    except FallbackError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/api/v1/slam/fallback_trigger")
def trigger_fallback_now():
    """Manually fires the currently configured fallback action right now
    (useful for testing without waiting for a real pose loss)."""
    try:
        result = fallback_controller.trigger("manual")
        return {"success": result.get("success", False), "result": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Fallback trigger failed: {e}")
