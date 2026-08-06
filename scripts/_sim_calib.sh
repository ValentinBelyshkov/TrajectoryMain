#!/bin/bash
cd /opt/main/Trajectory
PROJ=cc6494d4-2177-4812-b08e-e4cb4c2bd4e4
PF=/opt/main/Trajectory/Database/projects/$PROJ/procframe

pkill -f 'orb_slam3_ros2_wrapper|image_publish.py' 2>/dev/null
sleep 3
rm -f "$PF"/*.jpg "$PF"/*.txt 2>/dev/null
rm -f /tmp/terraslam_save_frames

source /opt/ros/humble/setup.bash
source /opt/main/Trajectory/host_colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/main/Trajectory/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY=:0

# Simulate manager: flag present, start SLAM, wait, start publisher (once loop)
echo "calibration_session=sim" > /tmp/terraslam_save_frames
ros2 run orb_slam3_ros2_wrapper mono \
  /opt/main/Trajectory/ORB_SLAM3/Vocabulary/ORBvoc.bin \
  /opt/main/Trajectory/Database/real.yaml --ros-args -r __ns:=/ > /tmp/sim_slam.log 2>&1 &
SLAM=$!
sleep 12
python3 -u /opt/main/Trajectory/Database/image_publish.py "$PROJ/frames" --fps 15 --procframe-dir "$PF" > /tmp/sim_pub.log 2>&1 &
PUB=$!

echo "=== tracking state over 40s ==="
for i in $(seq 1 20); do
  st=$(timeout 3 ros2 topic echo /orb_slam3/tracking_state --once 2>/dev/null | grep '^data:' | head -1)
  jpg=$(ls "$PF"/*.jpg 2>/dev/null | wc -l)
  echo "[t=$((i*2))s] state=$st procframe_jpg=$jpg"
  sleep 2
done
echo "=== final ==="
echo "procframe jpg=$(ls "$PF"/*.jpg 2>/dev/null | wc -l)"
echo "--- slam MONO stats / SaveFrame lines ---"
grep -iE 'SaveFrame triggered|MONO stats' /tmp/sim_slam.log | tail -8
kill $PUB $SLAM 2>/dev/null; pkill -f image_publish.py 2>/dev/null; pkill -f orb_slam3_ros2_wrapper 2>/dev/null
rm -f /tmp/terraslam_save_frames
