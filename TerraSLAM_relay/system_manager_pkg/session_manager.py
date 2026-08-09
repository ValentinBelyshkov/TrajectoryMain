"""
SLAM run sessions as background tasks with a persisted active session.

`slam_run` no longer blocks the HTTP request for the whole capture duration. It
returns a session id and the orchestration runs in the background. The active
session is persisted so that if the manager restarts mid-run, `maybe_resume_session`
finishes the stop-timer and brings the system back to the correct state (the
components themselves are restored by `ProcessManager.reconcile`).
"""
import asyncio
import os
import time
import uuid
from typing import Any, Dict, Optional

from .camera_snapshot import save_snapshot
from .process_manager import manager, update_yaml
from .state_store import state_store


def _projects_dir() -> str:
    return os.getenv("PROJECTS_DIR", "/home/orb/Database/projects")


async def run_slam_session(req) -> Dict[str, Any]:
    """Start a SLAM run in the background and return the session record."""
    session_id = uuid.uuid4().hex[:12]
    rec = {
        "id": session_id,
        "type": "slam_run",
        "project_id": req.project_id,
        "mode": req.mode,
        "duration": req.duration,
        "started_at": time.time(),
        "phase": "starting",
        "error": None,
    }
    state_store.set_session(rec)
    asyncio.create_task(_run(rec))
    return rec


async def _run(rec: Dict[str, Any]) -> None:
    session_id = rec["id"]
    project_id = rec["project_id"]
    mode = rec["mode"]
    publisher_name = f"publisher_{mode}"

    frames_dir = f"{_projects_dir()}/{project_id}/frames"
    procframe_dir = f"{_projects_dir()}/{project_id}/procframe"
    os.makedirs(procframe_dir, exist_ok=True)
    # The image publisher records every processed frame + pose here; start clean
    # so stale frames from a previous run don't mix in.
    for item in os.listdir(procframe_dir):
        try:
            os.remove(os.path.join(procframe_dir, item))
        except OSError:
            pass

    try:
        if mode == "folder" and not os.path.isdir(frames_dir):
            raise RuntimeError(f"Frames dir missing: {frames_dir}")

        yaml_path = "/opt/main/Trajectory/Database/real.yaml"
        # ORB-SLAM3 SaveAtlas() writes to "./<SaveAtlasToFile>.osa" relative to
        # the SLAM process CWD (/opt/main/Trajectory), so this must be relative
        # to that directory (no leading slash, no ".osa" suffix).
        save_path = f"Database/projects/{project_id}/calibrations/map"
        update_yaml(yaml_path, save_path)

        if mode == "folder":
            # The publisher loops over the frames dir and writes the processed
            # frames + pose txt into procframe (via PROCFRAME_DIR). It runs until
            # we stop it, because ORB-SLAM only tracks ~2 frames/sec.
            await manager.start(
                "publisher_folder",
                extra={"path": frames_dir, "procframe_dir": procframe_dir},
            )
        else:
            await manager.start("publisher_realsense")

        await asyncio.sleep(2)
        await manager.start("slam")

        rec["phase"] = "running"
        state_store.set_session(rec)

        # Let ORB-SLAM process frames for the requested duration. The publisher
        # (re)publishes frames in a loop and records each tracked frame itself.
        elapsed = 0.0
        while elapsed < rec["duration"]:
            await asyncio.sleep(1.0)
            elapsed += 1.0

        await manager.stop(publisher_name)
        await asyncio.sleep(2)
        await manager.stop("slam")

        # Ask ORB-SLAM to persist the built map as a .osa file in the project's
        # calibrations dir. slam_mode_manager (running alongside ORB-SLAM)
        # bridges this file flag into the ROS MapControl service.
        osa_path = f"{save_path}.osa"
        try:
            with open("/tmp/terraslam_slam_cmd", "w") as f:
                f.write("save_map\n")
            with open("/tmp/terraslam_slam_path", "w") as f:
                f.write(osa_path + "\n")
            rec["osa_file"] = osa_path
        except Exception as e:
            rec["osa_error"] = str(e)

        rec["phase"] = "done"
    except Exception as e:
        rec["error"] = str(e)
        rec["phase"] = "error"
        try:
            await manager.stop(publisher_name)
        except Exception:
            pass
        try:
            await manager.stop("slam")
        except Exception:
            pass
    finally:
        state_store.set_session(rec)


def get_session(session_id: str) -> Optional[Dict[str, Any]]:
    rec = state_store.get_session()
    if rec and rec.get("id") == session_id:
        return rec
    return None


async def maybe_resume_session() -> None:
    """Called on startup. If a SLAM session was in-flight when the manager
    restarted, recompute the remaining time and finish the stop sequence.

    The components are already restored by `manager.reconcile()` (they are
    marked desired while a session runs), so here we only need to honor the
    original duration and then stop them.
    """
    rec = state_store.get_session()
    if not rec:
        return
    if rec.get("phase") in ("done", "error"):
        state_store.clear_session()
        return

    elapsed = time.time() - rec.get("started_at", time.time())
    remaining = max(0.0, rec.get("duration", 0) - elapsed)
    mode = rec.get("mode", "folder")
    publisher_name = f"publisher_{mode}"

    rec["phase"] = "resuming"
    state_store.set_session(rec)

    await asyncio.sleep(remaining)
    try:
        await manager.stop(publisher_name)
    except Exception:
        pass
    await asyncio.sleep(2)
    try:
        await manager.stop("slam")
    except Exception:
        pass

    state_store.set_desired(publisher_name, False)
    state_store.set_desired("slam", False)

    rec["phase"] = "done"
    state_store.set_session(rec)
