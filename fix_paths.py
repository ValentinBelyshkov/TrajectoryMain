import pathlib

files = [
    "/opt/main/Trajectory/orb_slam3_ros2_wrapper/src/orb_slam3_interface.cpp",
    "/opt/main/Trajectory/entrypoint.sh",
    "/opt/main/Trajectory/docker-compose.yml",
]

replacements = {
    "/opt/main/Trajectory/orb_slam3_ros2_wrapper/src/orb_slam3_interface.cpp": [("/home/orb\";  ", "/opt/main/Trajectory\";  ")],
    "/opt/main/Trajectory/entrypoint.sh": [
        ("/home/orb/ORB_SLAM3/lib", "/opt/main/Trajectory/ORB_SLAM3/lib"),
        ("/home/orb/Database", "/opt/main/Trajectory/Database"),
    ],
    "/opt/main/Trajectory/docker-compose.yml": [
        ("/home/orb/TerraSLAM_relay", "/opt/main/Trajectory/TerraSLAM_relay"),
    ],
}

for filepath, reps in replacements.items():
    p = pathlib.Path(filepath)
    text = p.read_text()
    for old, new in reps:
        text = text.replace(old, new)
    p.write_text(text)
