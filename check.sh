#!/bin/bash
echo "=========================================="
echo "  TerraSLAM System Status Check"
echo "=========================================="

echo -e "\n📦 Container Status:"
docker ps | grep TerraSLAM

echo -e "\n🔄 Supervisor Processes:"
docker exec TerraSLAM /usr/bin/supervisorctl -s unix:///tmp/supervisor.sock status 2>&1

echo -e "\n📡 ROS2 Topics:"
docker exec TerraSLAM /bin/bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

echo -e "\n📊 Image Publisher Rate:"
docker exec TerraSLAM /bin/bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /camera/image_raw" 2>&1 | head -5

echo -e "\n📝 Recent Errors (if any):"
docker exec TerraSLAM cat /var/log/supervisor/*.err.log 2>&1 | tail -20

echo -e "\n=========================================="
echo "  Check Complete!"
echo "=========================================="
