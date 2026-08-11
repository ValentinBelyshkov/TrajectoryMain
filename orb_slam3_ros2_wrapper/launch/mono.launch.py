#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

# Force the ORB-SLAM3 Pangolin viewer onto the physical X session (:0).
# The node otherwise inherits DISPLAY from the launching terminal (often :1001),
# where the window is created but never shown.
os.environ["DISPLAY"] = ":0"
os.environ["XAUTHORITY"] = "/run/user/1000/gdm/Xauthority"
os.environ.setdefault("XDG_RUNTIME_DIR", "/tmp/runtime-root")
os.makedirs(os.environ["XDG_RUNTIME_DIR"], exist_ok=True)
os.chmod(os.environ["XDG_RUNTIME_DIR"], 0o700)

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    orb_wrapper_pkg = get_package_share_directory("orb_slam3_ros2_wrapper")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace", default_value="robot", description="The namespace of the robot"
    )

    orb_slam3_param_file = LaunchConfiguration("orb_slam3_param_file")
    declare_orb_slam3_param_file_cmd = DeclareLaunchArgument(
        name="orb_slam3_param_file",
        default_value="real.yaml", # gazebo_mono.yaml
        description="Path to the ORB-SLAM3 parameter file",
    )

    ros_params_file = LaunchConfiguration("ros_params_file")
    declare_ros_params_file_cmd = DeclareLaunchArgument(
        name="ros_params_file",
        default_value="euroc-mono-ros-params.yaml", # gazebo-mono-ros-params.yaml
        description="Path to the ROS2 parameters file",
    )

    def all_nodes_launch(context, robot_namespace):
        params_file = LaunchConfiguration("params_file")
        vocabulary_file_path = "/opt/main/Trajectory/ORB_SLAM3/Vocabulary/ORBvoc.bin"
        # NOTE: keep settings path consistent with existing launch files. Replace as needed.
        config_file_path = "/opt/main/Trajectory/Database/" + orb_slam3_param_file.perform(context)

        declare_params_file_cmd = DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(orb_wrapper_pkg, "params", "ros_params", ros_params_file.perform(context)),
            description="Full path to the ROS2 parameters file to use for all launched nodes",
        )

        # Optional namespacing hook (kept consistent with other launch files)
        base_frame = "" if robot_namespace.perform(context) == "" else robot_namespace.perform(context) + "/"
        param_substitutions = {
            # "robot_base_frame": base_frame + "base_footprint",
            # "odom_frame": base_frame + "odom",
        }

        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=robot_namespace.perform(context),
            param_rewrites=param_substitutions,
            convert_types=True,
        )

        orb_slam3_node = Node(
            package="orb_slam3_ros2_wrapper",
            executable="mono",
            output="screen",
            namespace=robot_namespace.perform(context),
            arguments=[vocabulary_file_path, config_file_path],
            parameters=[configured_params],
        )

        return [declare_params_file_cmd, orb_slam3_node]

    opaque_function = OpaqueFunction(function=all_nodes_launch, args=[robot_namespace])

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            declare_orb_slam3_param_file_cmd,
            declare_ros_params_file_cmd,
            robot_namespace_arg,
            opaque_function,
        ]
    )


