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

rm -f "$FLAG"
echo "=== TEST 1: flag ABSENT -> should save NOTHING ==="
ros2 run orb_slam3_ros2_wrapper mono \
  /opt/main/Trajectory/ORB_SLAM3/Vocabulary/ORBvoc.bin \
  /opt/main/Trajectory/Database/real.yaml --ros-args -r __ns:=/ > /tmp/s1.log 2>&1 &
SLAM=$!
sleep 15
python3 -u /opt/main/Trajectory/Database/image_publish.py "$PROJ/frames" --fps 15 --procframe-dir "$PF" > /tmp/p1.log 2>&1 &
sleep 20
echo "after test1 (no flag) procframe jpg = $(ls "$PF"/*.jpg 2>/dev/null | wc -l)"
kill %2 2>/dev/null; sleep 1

echo "=== TEST 2: flag PRESENT -> should save frames ==="
echo "calibration_session=test" > "$FLAG"
sleep 1
python3 -u /opt/main/Trajectory/Database/image_publish.py "$PROJ/frames" --fps 15 --procframe-dir "$PF" > /tmp/p2.log 2>&1 &
sleep 20
echo "after test2 (flag) procframe jpg = $(ls "$PF"/*.jpg 2>/dev/null | wc -l)"

echo "=== TEST 3: flag REMOVED mid-run -> saving stops ==="
rm -f "$FLAG"
sleep 12
echo "after test3 (flag removed) procframe jpg = $(ls "$PF"/*.jpg 2>/dev/null | wc -l)"

kill %1 2>/dev/null; pkill -f image_publish.py 2>/dev/null; pkill -f orb_slam3_ros2_wrapper 2>/dev/null
echo "=== SLAM save log lines ==="
grep -iE 'SaveFrame triggered|procframe' /tmp/s1.log | tail -8
echo DONE
