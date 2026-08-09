import asyncio
import logging
import os
import shutil
import time
from pathlib import Path
from typing import Literal, Optional, Union
from urllib.parse import urlparse

import httpx
from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from .routes_projects import (
    get_project_path,
    get_projects_root,
    read_project_metadata,
)
from .routes_components import component_action as _gateway_component_action
from .utils.yaml_editor import update_yaml
from .config import ValueReq as _ValueReq

router = APIRouter()
logger = logging.getLogger(__name__)

TERRASLAM_GATEWAY_URL = os.getenv("TERRASLAM_GATEWAY_URL", "http://127.0.0.1:9000")
PROJECTS_DIR = os.getenv("PROJECTS_DIR", "/opt/main/Trajectory/Database/projects")

COMPONENT_MAPPING = {
    "slam": "slam",
    "publisher": "publisher",
    "publisher:folder": "publisher_folder",
    "publisher:realsense": "publisher_realsense",
    "rosbridge": "rosbridge",
    "gps_bridge": "gps_bridge",
}

LOG_NAMES = {
    "slam": "slam_core",
    "publisher_folder": "publisher_folder",
    "publisher_realsense": "publisher_realsense",
    "gps_bridge": "gps_bridge",
    "rosbridge": "rosbridge",
}


class ComponentAction(BaseModel):
    component: str
    action: str
    project_id: Optional[str] = None
    save_frames: Optional[bool] = None


class CommandResponse(BaseModel):
    success: bool
    output: str
    error: Optional[str] = None


class ModeRequest(BaseModel):
    project_id: str
    mode: Literal["manual", "auto", "simulation", "hover"]


mode_states: dict = {}


@router.post("/mode")
async def set_mode(request: ModeRequest):
    """Switch drone operation mode."""
    mode_states[request.project_id] = request.mode
    return {
        "project_id": request.project_id,
        "mode": request.mode,
        "status": "mode_changed"
    }


@router.get("/mode/{project_id}")
async def get_mode(project_id: str):
    """Get current mode for a project."""
    return {
        "project_id": project_id,
        "mode": mode_states.get(project_id, "manual")
    }


@router.post("/emergency-stop")
async def emergency_stop(project_id: str):
    """Emergency stop for drone."""
    mode_states[project_id] = "emergency_stop"
    return {
        "project_id": project_id,
        "status": "emergency_stopped",
        "message": "All motors stopped"
    }


@router.post("/arm")
async def arm_drone(project_id: str):
    """Arm drone for flight."""
    return {
        "project_id": project_id,
        "status": "armed",
        "message": "Drone is armed and ready"
    }


@router.post("/disarm")
async def disarm_drone(project_id: str):
    """Disarm drone."""
    return {
        "project_id": project_id,
        "status": "disarmed",
        "message": "Drone is disarmed"
    }


def _gateway_client() -> httpx.AsyncClient:
    return httpx.AsyncClient(base_url=TERRASLAM_GATEWAY_URL, timeout=30.0)


def _is_loopback_gateway() -> bool:
    """True when the configured gateway is this same process."""
    try:
        host = urlparse(TERRASLAM_GATEWAY_URL).hostname or ""
    except Exception:
        return False
    return (
        host in ("localhost", "127.0.0.1", "::1")
        or host == os.getenv("HOSTNAME")
    )


async def _call_component_action(component: str, action: str, value: Optional[str] = None) -> list[str]:
    """Invoke a component action. When the gateway points at this same
    process we call the FastAPI handler directly (no self-HTTP, no DNS needed
    so it works from the LAN host too). Otherwise fall back to HTTP."""
    if _is_loopback_gateway():
        try:
            result = await _gateway_component_action(
                component, action, _ValueReq(value=value)
            )
            return result.get("results", []) or [str(result)]
        except HTTPException as e:
            return [f"{component}: error: {e.detail}"]
        except Exception as e:  # pragma: no cover - defensive
            return [f"{component}: error: {e}"]

    async with _gateway_client() as client:
        r = await client.post(
            f"/api/v1/components/{component}/{action}",
            json={"value": value} if value else {},
        )
        r.raise_for_status()
        data = r.json()
    results = data.get("results", [])
    return results if isinstance(results, list) else [str(results)]


async def _gateway_health() -> dict:
    """Check gateway health, returns empty dict on error."""
    async with _gateway_client() as client:
        try:
            r = await client.get("/health")
            if r.status_code == 200:
                return r.json()
        except Exception:
            pass
    return {}


def _resolve_component(comp: str, project_type: Optional[str] = None) -> str:
    """Map frontend component alias to gateway component name."""
    mapped = COMPONENT_MAPPING.get(comp, comp)
    if comp == "publisher" and project_type:
        return "publisher_folder" if project_type == "симуляция" else "publisher_realsense"
    return mapped


async def _gateway_action(component: str, action: str, value: Optional[str] = None) -> list[str]:
    """Call gateway /api/v1/components/{component}/{action} and return list of result strings."""
    return await _call_component_action(component, action, value=value)


async def _gateway_status() -> dict:
    """Fetch gateway status. Returns raw gateway response or empty on error."""
    async with _gateway_client() as client:
        try:
            r = await client.get("/api/v1/status")
            if r.status_code == 200:
                return r.json()
        except Exception:
            pass
    return {}


@router.post("/terraslam/component")
async def control_terraslam_component(action: ComponentAction, request: Request) -> CommandResponse:
    """Control TerraSLAM components via HTTP gateway."""
    valid_actions = {"start", "stop", "restart", "status", "kill"}
    valid_components = set(COMPONENT_MAPPING.keys()) | {"all"}

    if action.component not in valid_components:
        raise HTTPException(400, f"Invalid component: {action.component}")
    if action.action not in valid_actions:
        raise HTTPException(400, f"Invalid action: {action.action}")

    projects_root = get_projects_root(request)
    project = None
    if action.project_id:
        project = read_project_metadata(projects_root, action.project_id)

    project_type = project.type if project else None

    if action.action in ("start", "restart") and action.component in ("slam", "all") and action.project_id:
        yaml_path = os.getenv("YAML_PATH", "/app/trajectory-db/real.yaml")
        load_path = None
        save_path = None
        if project and project.calibration_status == "calibrated":
            load_path = f"{PROJECTS_DIR}/{action.project_id}/calibrations/map"
            save_path = f"{PROJECTS_DIR}/{action.project_id}/calibrations/map"
        logger.info(
            f"Updating SLAM YAML before {action.action}: load_filename={load_path}, save_filename={save_path}"
        )
        update_yaml(yaml_path, save_filename=save_path, load_filename=load_path)

    if action.component == "all" and action.action in ("start", "restart", "stop", "kill"):
        if project_type == "симуляция":
            target_components = ["slam", "publisher_folder", "rosbridge"]
            others = ["publisher_realsense"]
        else:
            target_components = ["slam", "publisher_realsense", "rosbridge"]
            others = ["publisher_folder"]

        combined_output = ""
        success = True

        for comp in others:
            try:
                results = await _gateway_action(comp, "kill")
                combined_output += "\n".join(results) + "\n"
            except Exception as e:
                combined_output += f"{comp}: kill error {e}\n"

        for comp in target_components:
            try:
                if comp.startswith("publisher_") and action.action in ("start", "restart"):
                    if action.project_id:
                        frames_dir = f"{PROJECTS_DIR}/{action.project_id}/frames"
                        os.makedirs(frames_dir, exist_ok=True)
                        # publisher_folder expects the path via the dedicated
                        # /api/v1/publisher/path endpoint; publisher_realsense
                        # receives it as the component start value (--frames-dir).
                        if comp == "publisher_folder":
                            pub_res = await _call_component_action("publisher_folder", "start", value=frames_dir)
                            combined_output += "\n".join(pub_res) + "\n"
                            continue
                    else:
                        combined_output += f"{comp}: skipped (no project_id)\n"
                        success = False
                        continue

                if action.action == "restart":
                    kill_res = await _gateway_action(comp, "kill")
                    combined_output += "\n".join(kill_res) + "\n"
                    start_value = None
                    if comp == "publisher_realsense":
                        if action.project_id and frames_dir:
                            save_frames = action.save_frames if action.save_frames is not None else True
                            start_value = frames_dir if save_frames else "__nosave__"
                    start_res = await _gateway_action(comp, "start", value=start_value)
                    combined_output += "\n".join(start_res) + "\n"
                else:
                    results = await _gateway_action(comp, action.action)
                    combined_output += "\n".join(results) + "\n"
            except Exception as e:
                combined_output += f"{comp}: error {e}\n"
                success = False

        return CommandResponse(success=success, output=combined_output)

    gateway_component = _resolve_component(action.component, project_type)

    if gateway_component.startswith("publisher_") and action.action in ("start", "restart"):
        if not action.project_id:
            raise HTTPException(400, "project_id is required to start publisher")
        frames_dir = f"{PROJECTS_DIR}/{action.project_id}/frames"
        os.makedirs(frames_dir, exist_ok=True)
        if gateway_component == "publisher_folder":
            # publisher_folder expects the path via the dedicated endpoint
            async with _gateway_client() as client:
                r = await client.post("/api/v1/publisher/path", json={"path": frames_dir})
                r.raise_for_status()
            return CommandResponse(success=True, output="publisher_folder path set")
        # publisher_realsense receives the frames dir as its start value
        try:
            results = await _call_component_action(
                "publisher_realsense", action.action, value=frames_dir
            )
            return CommandResponse(success=True, output="\n".join(results))
        except Exception as e:
            raise HTTPException(500, str(e))

    try:
        if action.action == "restart":
            kill_res = await _gateway_action(gateway_component, "kill")
            start_res = await _gateway_action(gateway_component, "start")
            output = "\n".join(kill_res + start_res)
        else:
            results = await _gateway_action(gateway_component, action.action)
            output = "\n".join(results)
        return CommandResponse(success=True, output=output)
    except Exception as e:
        raise HTTPException(500, str(e))


@router.get("/terraslam/status")
async def get_terraslam_status():
    """Get detailed status of all TerraSLAM components from gateway."""
    gateway_data = await _gateway_status()
    if not gateway_data:
        return {
            "system_status": "not_working",
            "error": "Gateway unavailable",
            "components": {},
            "publisher_mode": "unknown",
            "orphaned_processes": {},
            "supervisor_output": ""
        }

    components = gateway_data.get("components", [])
    components_status: dict[str, str] = {}
    publisher_mode = "folder"
    for comp in components:
        name = comp.get("name", "")
        short_name = name.split("/")[-1] if "/" in name else name
        components_status[short_name] = comp.get("message", "UNKNOWN")
        values = comp.get("values", {})
        if "publisher_mode" in values:
            publisher_mode = values["publisher_mode"]

    main_components = ["slam"]
    if publisher_mode == "folder":
        main_components.append("publisher_folder")
    else:
        main_components.append("publisher_realsense")
        main_components.append("rosbridge")

    all_running = all(
        components_status.get(comp, "").upper() == "RUNNING"
        for comp in main_components
    )

    orphaned_processes: dict[str, int] = {}
    for comp_name, state in components_status.items():
        if comp_name not in main_components and state.upper() == "RUNNING":
            orphaned_processes[comp_name] = 1

    if all_running and not orphaned_processes:
        system_status = "working"
    elif orphaned_processes:
        system_status = "warning"
    else:
        system_status = "not_working"

    return {
        "system_status": system_status,
        "components": components_status,
        "publisher_mode": publisher_mode,
        "orphaned_processes": orphaned_processes,
        "supervisor_output": ""
    }


@router.get("/terraslam/logs/{component}")
async def get_terraslam_logs(component: str, lines: int = 50):
    """Get recent logs for a specific component from gateway."""
    gateway_component = _resolve_component(component)

    try:
        async with _gateway_client() as client:
            params = {"component": gateway_component, "limit": lines}
            r = await client.get("/api/v1/logs", params=params)
            if r.status_code != 200:
                raise HTTPException(503, "Gateway logs unavailable")
            data = r.json()

        logs = data.get("logs", [])
        stderr_lines = []
        stdout_lines = []
        for entry in logs:
            src = entry.get("source", "stdout")
            msg = entry.get("message", "")
            if src == "stderr":
                stderr_lines.append(msg)
            else:
                stdout_lines.append(msg)

        status_indicators = {
            "tracking_lost": False,
            "not_initialized": False,
            "initializing": False,
            "valid_data": False
        }
        for line in stderr_lines:
            if "-3.0" in line and "tracking" in line.lower():
                status_indicators["tracking_lost"] = True
            elif "-1.0" in line and "init" in line.lower():
                status_indicators["not_initialized"] = True
            elif "initializing" in line.lower() or "waiting" in line.lower():
                status_indicators["initializing"] = True
            elif any(c.isdigit() for c in line) and "lat" in line.lower():
                status_indicators["valid_data"] = True

        return {
            "component": component,
            "stderr_logs": "\n".join(stderr_lines),
            "stdout_logs": "\n".join(stdout_lines),
            "status_indicators": status_indicators
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(500, str(e))


@router.get("/terraslam/health")
async def terraslam_health():
    """Check TerraSLAM gateway health."""
    health = await _gateway_health()
    if not health:
        return {"status": "unhealthy", "container_status": "gateway_unavailable"}
    if health.get("gateway_ready") and health.get("status_fresh"):
        return {"status": "healthy", "container_status": "running"}
    return {"status": "unhealthy", "container_status": "degraded"}


@router.post("/terraslam/slam/test-run")
async def slam_test_run(request: Request, project_id: Optional[str] = None):
    t_start = time.time()
    print(f"[TEST-RUN] Start. project_id={project_id}")

    try:
        projects_root = get_projects_root(request)
        project_path = get_project_path(projects_root, project_id)
        
        procframe_dir = project_path / "procframe"
        if procframe_dir.exists():
            print(f"[TEST-RUN] Clearing procframe directory: {procframe_dir}")
            for item in procframe_dir.iterdir():
                try:
                    if item.is_file():
                        item.unlink()
                    elif item.is_dir():
                        shutil.rmtree(item)
                except Exception as e:
                    print(f"[TEST-RUN] Warning: Could not delete {item}: {e}")
            print("[TEST-RUN] procframe directory cleared")
        else:
            procframe_dir.mkdir(parents=True, exist_ok=True)
            print(f"[TEST-RUN] procframe directory created: {procframe_dir}")

        project = read_project_metadata(projects_root, project_id)
        publisher_mode = "folder" if (project and project.type == "симуляция") else "realsense"
        print(f"[TEST-RUN] Publisher mode: {publisher_mode}")

        print("[TEST-RUN] Starting SLAM run via TerraSLAM...")
        try:
            async with httpx.AsyncClient(timeout=300.0) as client:
                res = await client.post(
                    f"{TERRASLAM_GATEWAY_URL}/slam/run",
                    json={
                        "project_id": project_id,
                        "mode": publisher_mode,
                        "duration": 15
                    }
                )
                result = res.json()
            print(f"[TEST-RUN] TerraSLAM response: {result}")
        except Exception as e:
            print(f"[TEST-RUN] ERROR contacting gateway {TERRASLAM_GATEWAY_URL}: {e}")
            return CommandResponse(
                success=False, output="",
                error=f"Gateway {TERRASLAM_GATEWAY_URL} unreachable: {e}",
            )

        if not result.get("success"):
            error = result.get("results", ["Unknown error"])[0]
            print(f"[TEST-RUN] ERROR: {error}")
            return CommandResponse(success=False, output="", error=str(error))

        osa_file = result.get("osa_file")
        if osa_file:
            print(f"[TEST-RUN] OSA file created: {osa_file}")
        else:
            print("[TEST-RUN] WARNING: No OSA file in response")

        calibrations_dir = project_path / "calibrations"
        expected_file = calibrations_dir / "map.osa"
        
        for _ in range(10):
            if expected_file.exists():
                break
            await asyncio.sleep(1)
            print(f"[TEST-RUN] Waiting for .osa file...")

        if not expected_file.exists():
            recent = [f for f in calibrations_dir.glob("*.osa") if f.stat().st_mtime > time.time() - 120]
            if recent:
                expected_file = recent[0]
                print(f"[TEST-RUN] Found recent: {expected_file}")
            else:
                print("[TEST-RUN] ERROR: No .osa file found locally!")
                return CommandResponse(success=False, output="", error="No .osa created")

        print(f"[TEST-RUN] Done in {time.time() - t_start:.1f}s")
        return CommandResponse(success=True, output="", error=None)

    except Exception as e:
        print(f"[TEST-RUN] FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        raise HTTPException(500, str(e))
