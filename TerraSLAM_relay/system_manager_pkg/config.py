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
            'source /opt/ros/humble/setup.bash && '
            'source /opt/main/Trajectory/host_colcon_ws/install/setup.bash && '
            'export PATH="/usr/bin:/bin:/usr/local/bin:/opt/ros/humble/bin:$PATH" && '
            'export LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/main/Trajectory/lib:$LD_LIBRARY_PATH" && '
            'mkdir -p /opt/main/Trajectory/.ros/log && '
            'ros2 run orb_slam3_ros2_wrapper mono '
            '/opt/main/Trajectory/ORB_SLAM3/Vocabulary/ORBvoc.bin '
            '/opt/main/Trajectory/Database/real.yaml --ros-args -r __ns:=/'
        ],
        "cwd": "/opt/main/Trajectory",
        "env": {"ROS_DOMAIN_ID": "0", "QT_X11_NO_MITSHM": "1"},
        "autorestart": False,
        "max_restarts": 0,
    },
    "publisher_folder": {
        "cwd": "/opt/main/Trajectory/Database",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": False,
        "max_restarts": 0,
    },
    "relay": {
        "cwd": "/opt/main/Trajectory/TerraSLAM_relay",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": False,
        "max_restarts": 0,
    },
    "publisher_realsense": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && source /opt/main/Trajectory/host_colcon_ws/install/setup.bash && exec python3 /opt/main/Trajectory/Database/realsense.py"
        ],
        "cwd": "/opt/main/Trajectory/Database",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": False,
        "max_restarts": 0,
    },
    "gps_bridge": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && source /opt/main/Trajectory/host_colcon_ws/install/setup.bash && "
            "python3 /opt/main/Trajectory/TerraSLAM_relay/Serial/gps_bridge_node.py --ros-args "
            "-p protocol:=msp -p hw_type:=uart -p port:=/dev/ttyTHS1 -p baudrate:=115200"
        ],
        "cwd": "/opt/main/Trajectory/TerraSLAM_relay/Serial",
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
        "cwd": "/opt/main/Trajectory",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": True,
        "max_restarts": 999,
        "restart_window": 60,
    },
    "slam_mode_manager": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && "
            "source /opt/main/Trajectory/host_colcon_ws/install/setup.bash && "
            "python3 /opt/main/Trajectory/TerraSLAM_relay/slam_mode_manager.py"
        ],
        "cwd": "/opt/main/Trajectory",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": True,
        "max_restarts": 5,
        "restart_window": 60,
    },
}

# --- Pose liveness config ---
POSE_TOPIC = "/camera_pose"
TRACKING_STATE_TOPIC = "/orb_slam3/tracking_state"
POSE_STALE_AFTER_SECONDS = 3.0
POSE_POLL_INTERVAL_SECONDS = 1.0
# NOTE: pose/tracking-state are now read via a persistent rclpy subscriber
# (system_manager_pkg/ros_pose_subscriber.py) instead of spawning
# `ros2 topic echo --once` per poll, so no echo timeout is needed anymore.
# Kept here only in case older code still imports it.
POSE_ECHO_TIMEOUT_SECONDS = 3.0

# ORB-SLAM3 tracking state values published on /orb_slam3/tracking_state
# (std_msgs/Int8): -1=SYSTEM_NOT_READY, 0=NO_IMAGES_YET, 1=NOT_INITIALIZED,
# 2=OK, 3=RECENTLY_LOST, 4=LOST
TRACKING_STATE_LOST_VALUES = (3, 4)
TRACKING_STATE_OK_VALUE = 2

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
