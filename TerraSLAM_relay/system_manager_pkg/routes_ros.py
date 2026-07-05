"""
Dashboard HTML route, direct ROS2 CLI passthrough endpoints (topics, nodes,
mavros arm/set_mode), and the legacy /api/sm/... + /api/ros2/... compatibility
aliases kept for the existing dashboard JS.
"""
import os

from fastapi import APIRouter, HTTPException
from fastapi.responses import HTMLResponse
from typing import Optional

from .config import ArmReq, FlightModeReq, ModeReq, PathReq, ValueReq
from .process_manager import manager
from .ros_utils import ros2_run
from . import routes_components as rc
from . import routes_slam as rs

router = APIRouter()

_DASHBOARD_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "index.html")

ROS2_TOPICS = {
    "covariance": "/orb_slam3/covariance",
    "camera_pose": "/orb_slam3/camera_pose",
    "imu": "/imu/data",
    "mavros_state": "/mavros/state",
    "mavros_imu": "/mavros/imu/data",
    "mavros_pose": "/mavros/local_position/pose",
    "tracking": "/orb_slam3/tracking_state",
}


@router.get("/", response_class=HTMLResponse)
def dashboard():
    try:
        with open(_DASHBOARD_PATH, "r", encoding="utf-8") as f:
            return HTMLResponse(content=f.read())
    except FileNotFoundError:
        return HTMLResponse(content="<h1>index.html not found</h1>", status_code=404)


@router.get("/api/v1/ros/topics")
def ros2_topic_list():
    stdout, stderr, rc_code = ros2_run(["topic", "list"])
    topics = [t.strip() for t in stdout.splitlines() if t.strip()] if rc_code == 0 else []
    return {"topics": topics, "error": stderr, "returncode": rc_code}


@router.get("/api/v1/ros/nodes")
def ros2_node_list():
    stdout, stderr, rc_code = ros2_run(["node", "list"])
    nodes = [n.strip() for n in stdout.splitlines() if n.strip()] if rc_code == 0 else []
    return {"nodes": nodes, "error": stderr, "returncode": rc_code}


@router.get("/api/v1/ros/topic/{name}")
def ros2_topic_echo(name: str):
    topic = ROS2_TOPICS.get(name)
    if not topic:
        raise HTTPException(400, detail=f"Unknown topic '{name}'. Known: {list(ROS2_TOPICS.keys())}")
    stdout, stderr, rc_code = ros2_run(["topic", "echo", "--once", topic], timeout=5)
    return {"topic": topic, "output": stdout, "error": stderr, "returncode": rc_code}


@router.post("/api/v1/ros/mavros/arm")
def ros2_mavros_arm(req: ArmReq):
    val = "true" if req.arm else "false"
    stdout, stderr, rc_code = ros2_run(
        ["service", "call", "/mavros/cmd/arming", "mavros_msgs/srv/CommandBool", f"{{value: {val}}}"],
        timeout=8,
    )
    return {"output": stdout, "error": stderr, "returncode": rc_code}


@router.post("/api/v1/ros/mavros/set_mode")
def ros2_mavros_set_mode(req: FlightModeReq):
    stdout, stderr, rc_code = ros2_run(
        ["service", "call", "/mavros/set_mode", "mavros_msgs/srv/SetMode", f'{{custom_mode: "{req.mode}"}}'],
        timeout=8,
    )
    return {"output": stdout, "error": stderr, "returncode": rc_code}


# --- Compatibility Aliases for Dashboard (index.html) ---

@router.get("/api/sm/status")
def sm_status_alias():
    return rc.get_status()


@router.post("/api/sm/components/{component}/{action}")
async def sm_component_action_alias(component: str, action: str, req: Optional[ValueReq] = None):
    return await rc.component_action(component, action, req)


@router.get("/api/sm/slam/status")
def sm_slam_status_alias():
    return rs.get_slam_status()


@router.post("/api/sm/slam/mode")
async def sm_slam_mode_alias(req: ModeReq):
    return await rs.set_slam_mode(req)


@router.post("/api/sm/slam/command")
async def sm_slam_command_alias(req: ValueReq):
    return await rs.slam_command(req)


@router.post("/api/sm/publisher/mode")
async def sm_publisher_mode_alias(req: ModeReq):
    return await rc.pub_mode(req)


@router.post("/api/sm/publisher/path")
async def sm_publisher_path_alias(req: PathReq):
    return await rc.pub_path(req)


@router.get("/api/sm/logs")
def sm_logs_alias(component: Optional[str] = None, limit: int = 100, since: Optional[float] = None):
    return rc.get_logs(component, limit, since)


@router.get("/api/ros2/topic/{name}")
def ros2_topic_echo_alias(name: str):
    return ros2_topic_echo(name)


@router.post("/api/ros2/mavros/arm")
def ros2_mavros_arm_alias(req: ArmReq):
    return ros2_mavros_arm(req)


@router.post("/api/ros2/mavros/set_mode")
def ros2_mavros_set_mode_alias(req: FlightModeReq):
    return ros2_mavros_set_mode(req)


@router.post("/api/ros2/slam/start")
async def ros2_slam_start_alias():
    out = await manager.start("slam")
    return {"success": True, "message": out}


@router.post("/stop/slam")
async def stop_slam_legacy_alias():
    out = await manager.stop("slam")
    return {"success": True, "message": out}
