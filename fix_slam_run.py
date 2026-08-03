import pathlib

p = pathlib.Path("/opt/main/Trajectory/TerraSLAM_relay/system_manager_pkg/routes_slam.py")
text = p.read_text()

# Add import for camera_snapshot
old_imports = """from .config import (
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
from .ros_utils import ros2_run"""

new_imports = """from .config import (
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
from .camera_snapshot import save_snapshot"""

text = text.replace(old_imports, new_imports)

# Replace the simple sleep with photo capture loop
old_sleep = """    await asyncio.sleep(req.duration)

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
        results.append(f"slam stop error: {e}")"""

new_sleep = """    procframe_dir = f"/opt/main/Trajectory/Database/projects/{req.project_id}/procframe"
    os.makedirs(procframe_dir, exist_ok=True)

    snap_interval = 1.0
    snap_count = 0
    snap_errors = 0
    elapsed = 0.0
    while elapsed < req.duration:
        await asyncio.sleep(snap_interval)
        elapsed += snap_interval
        snap_count += 1
        snap_path = os.path.join(procframe_dir, f"calib_{snap_count:04d}.jpg")
        try:
            snap = save_snapshot(snap_path, timeout=5.0)
            if not snap.get("success"):
                snap_errors += 1
                results.append(f"snapshot error #{snap_count}: {snap.get('error', 'unknown')}")
        except Exception as e:
            snap_errors += 1
            results.append(f"snapshot exception #{snap_count}: {e}")

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

    results.append(f"snapshots: {snap_count} saved, {snap_errors} errors")"""

text = text.replace(old_sleep, new_sleep)

p.write_text(text)
