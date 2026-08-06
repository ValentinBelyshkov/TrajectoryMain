#!/bin/bash
cd /opt/main/Trajectory
python3 -c "import ast; ast.parse(open('TerraSLAM_relay/system_manager_pkg/routes_calibration.py').read()); print('routes_calibration.py: syntax OK')"
echo "--- manager running? ---"
ps aux | grep system_manager.py | grep -v grep | head -1
echo "--- flag present? ---"
ls -la /tmp/terraslam_save_frames 2>&1 || echo "no flag (good - not in calibration)"
echo "--- grep flag usage in calibration ---"
grep -n '_enable_frame_saving\|_disable_frame_saving\|SAVE_FRAMES_FLAG' TerraSLAM_relay/system_manager_pkg/routes_calibration.py
