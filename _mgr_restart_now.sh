#!/bin/bash
# Restart the TerraSLAM manager cleanly and detached from this SSH session.
set +e
echo "=== $(date) stopping components & manager ==="
pkill -f "system_manager.py" 2>/dev/null
pkill -f "gps_client.py" 2>/dev/null
pkill -f "gps_bridge_node.py" 2>/dev/null
pkill -f "gps_filter_node.py" 2>/dev/null
pkill -f "slam_mode_manager.py" 2>/dev/null
pkill -f "orb_slam3_ros2_wrapper" 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null
pkill -f "realsense.py" 2>/dev/null
pkill -f "image_publish.py" 2>/dev/null
pkill -f "rosbridge" 2>/dev/null
sleep 3
echo "=== remaining related procs (should be none) ==="
pgrep -af "system_manager.py|gps_client|gps_bridge|gps_filter|slam_mode|orb_slam3|realsense|rosbridge|ros2 launch" || echo "none"
echo "=== spawning manager (detached via _mgr_daemon.py) ==="
python3 /opt/main/Trajectory/_mgr_daemon.py
sleep 7
echo "=== port 9000 ==="
(ss -ltnp 2>/dev/null | grep ':9000') || echo "not listening"
curl -s -o /dev/null -w "manager health http=%{http_code}\n" http://127.0.0.1:9000/health 2>&1 || echo "manager down"
echo "=== END ==="
