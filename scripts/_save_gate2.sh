#!/bin/bash
PROJ=/opt/main/Trajectory/Database/projects/cc6494d4-2177-4812-b08e-e4cb4c2bd4e4
PF=$PROJ/procframe
FLAG=/tmp/terraslam_save_frames

pkill -f 'orb_slam3_ros2_wrapper|image_publish.py' 2>/dev/null
sleep 2
rm -f "$PF"/*.jpg "$PF"/*.txt 2>/dev/null

source /opt/ros/humble/setup.bash
source /opt/main/Trajectory/host_colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/main/Trajectory/lib:${LD_LIBRARY_PATH:-}"
export DISPLAY=:0

echo "=== flag PRESENT, long enough for tracking to reach OK ==="
echo "calibration_session=test" > "$FLAG"
ros2 run orb_slam3_ros2_wrapper mono \
  /opt/main/Trajectory/ORB_SLAM3/Vocabulary/ORBvoc.bin \
  /opt/main/Trajectory/Database/real.yaml --ros-args -r __ns:=/ > /tmp/s.log 2>&1 &
SLAM=$!
sleep 12
python3 -u /opt/main/Trajectory/Database/image_publish.py "$PROJ/frames" --fps 15 --procframe-dir "$PF" > /tmp/p.log 2>&1 &
PUB=$!
sleep 45
echo "procframe jpg while flag present = $(ls "$PF"/*.jpg 2>/dev/null | wc -l)"
echo "tracking states seen:"; grep -oE 'state=[0-9]' /tmp/s.log | sort | uniq -c

echo "=== remove flag, keep running 12s ==="
rm -f "$FLAG"
S1=$(ls "$PF"/*.jpg 2>/dev/null | wc -l)
sleep 12
S2=$(ls "$PF"/*.jpg 2>/dev/null | wc -l)
echo "procframe jpg before=$S1 after=$S2 (should be equal => saving stopped)"

kill $PUB $SLAM 2>/dev/null; pkill -f image_publish.py 2>/dev/null; pkill -f orb_slam3_ros2_wrapper 2>/dev/null
echo DONE
