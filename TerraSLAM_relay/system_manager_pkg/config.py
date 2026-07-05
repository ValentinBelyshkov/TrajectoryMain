"""
Static configuration for the TerraSLAM System Manager: component process
definitions, request/response models, and small filesystem constants.
"""
from typing import Optional

from pydantic import BaseModel

# --- Config ---
DEFAULT_MODE = "folder"
PUBLISHER_MODE_FILE = "/tmp/terraslam_publisher_mode"

COMPONENT_CONFIG = {
    "slam": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && "
            "source /home/orb/colcon_ws/install/setup.bash && "
            "export PATH='/usr/bin:/bin:/opt/ros/humble/bin:$PATH' && "
            "export LD_LIBRARY_PATH='/home/orb/ORB_SLAM3/lib:/home/orb/colcon_ws/install/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH' && "
            "mkdir -p /home/orb/.ros/log && "
            "ros2 run orb_slam3_ros2_wrapper mono /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.bin /home/orb/Database/real.yaml --ros-args -r __ns:=/"
        ],
        "cwd": "/home/orb",
        "env": {"ROS_DOMAIN_ID": "0", "QT_X11_NO_MITSHM": "1"},
        "autorestart": False,
        "max_restarts": 0,
    },
    "publisher_folder": {
        "cwd": "/home/orb/Database",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": False,
        "max_restarts": 0,
    },
    "relay": {
        "cwd": "/home/orb/TerraSLAM_relay",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": False,
        "max_restarts": 0,
    },
    "publisher_realsense": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && source /home/orb/colcon_ws/install/setup.bash && exec python3 /home/orb/Database/realsense.py"
        ],
        "cwd": "/home/orb/Database",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": False,
        "max_restarts": 0,
    },
    "gps_bridge": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && source /home/orb/colcon_ws/install/setup.bash && "
            "python3 /home/orb/TerraSLAM_relay/Serial/gps_bridge_node.py --ros-args "
            "-p protocol:=msp -p hw_type:=uart -p port:=/dev/ttyCH341USB0 -p baudrate:=115200"
        ],
        "cwd": "/home/orb/TerraSLAM_relay/Serial",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": True,
        "max_restarts": 3,
        "restart_window": 60,
    },
    "rosbridge": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && "
            "ros2 launch rosbridge_server rosbridge_websocket_launch.xml "
            "port:=9091 cors_origin:=* max_message_size:=10000000 fragment_timeout:=4000"
        ],
        "cwd": "/home/orb",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": True,
        "max_restarts": 999,
        "restart_window": 60,
    },
    "slam_mode_manager": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && "
            "source /home/orb/colcon_ws/install/setup.bash && "
            "python3 /home/orb/TerraSLAM_relay/slam_mode_manager.py"
        ],
        "cwd": "/home/orb",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": True,
        "max_restarts": 5,
        "restart_window": 60,
    },
}

# --- Pose liveness config ---
POSE_TOPIC = "/orb_slam3/camera_pose"
POSE_STALE_AFTER_SECONDS = 3.0
POSE_POLL_INTERVAL_SECONDS = 1.0

# ORB-SLAM3 publishes this sentinel position when tracking is lost:
# position.x = position.y = position.z = -3.0
TRACKING_LOST_SENTINEL = -3.0
TRACKING_LOST_EPSILON = 1e-6

# --- Fallback (failsafe) config ---
# Fired when the pose topic goes stale OR the tracking-lost sentinel pose
# is seen. The configured method is sent to the flight controller over
# MAVLink as a flight-mode change.
VALID_FALLBACK_METHODS = ["POSHOLD", "RTL", "LAND"]
DEFAULT_FALLBACK_METHOD = "RTL"
DEFAULT_FALLBACK_CONNECTION = "udp:127.0.0.1:14550"


# --- Pydantic Models ---
class ValueReq(BaseModel):
    value: Optional[str] = None


class ModeReq(BaseModel):
    mode: str


class PathReq(BaseModel):
    path: str


class CalibPathReq(BaseModel):
    calib_path: str


class RunReq(BaseModel):
    project_id: str
    mode: str = "folder"
    duration: int = 15


class ArmReq(BaseModel):
    arm: bool = True


class FlightModeReq(BaseModel):
    mode: str


class HardwareCalibrateReq(BaseModel):
    connection: str = "udp:127.0.0.1:14550"
    timeout: float = 15.0


class FallbackMethodReq(BaseModel):
    method: str


class FallbackConnectionReq(BaseModel):
    connection: str
