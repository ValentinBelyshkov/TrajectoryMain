#!/bin/bash
cd /opt/main/Trajectory
pkill -f 'orb_slam3_ros2_wrapper|image_publish.py' 2>/dev/null
sleep 2
source /opt/ros/humble/setup.bash
source /opt/main/Trajectory/host_colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=0 DISPLAY=:0
export LD_LIBRARY_PATH='/opt/ros/humble/lib:/opt/main/Trajectory/lib'
echo 'calib' > /tmp/terraslam_save_frames
ros2 run orb_slam3_ros2_wrapper mono \
  /opt/main/Trajectory/ORB_SLAM3/Vocabulary/ORBvoc.bin \
  /opt/main/Trajectory/Database/real.yaml --ros-args -r __ns:=/ > /tmp/sim_slam.log 2>&1 &
SLAM=$!
sleep 14
if kill -0 $SLAM 2>/dev/null; then echo "SLAM ALIVE pid=$SLAM"; else echo "SLAM DEAD"; fi
echo "--- topics ---"
timeout 5 ros2 topic list 2>/dev/null | grep -iE 'tracking|image_raw|pose' 
echo "--- slam log tail ---"
tail -20 /tmp/sim_slam.log
pkill -f 'orb_slam3_ros2_wrapper' 2>/dev/null
rm -f /tmp/terraslam_save_frames
