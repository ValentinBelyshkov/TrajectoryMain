#!/bin/bash
echo "=== Final Verification ==="

echo -e "\n1. Running Publisher Processes:"
docker exec TerraSLAM ps aux | grep image_publish | grep -v grep || echo "None found"

echo -e "\n2. Supervisor Status:"
docker exec TerraSLAM /usr/bin/supervisorctl -s unix:///tmp/supervisor.sock status | grep publisher

echo -e "\n3. Topic Info (should show 2 publishers):"
docker exec TerraSLAM /bin/bash -l -c "source /opt/ros/humble/setup.bash && ros2 topic info /camera/image_raw"

echo -e "\n4. Frame Rate (should be ~30 Hz):"
docker exec TerraSLAM /bin/bash -l -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic hz /camera/image_raw" 2>&1 | head -5

echo -e "\n5. SLAM Subscription:"
docker exec TerraSLAM /bin/bash -l -c "source /opt/ros/humble/setup.bash && ros2 topic info /camera/image_raw --verbose" | grep -A 2 "Subscription"

echo -e "\n=== Verification Complete ==="
