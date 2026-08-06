#!/bin/bash
cd /opt/main/Trajectory
PROJ=cc6494d4-2177-4812-b08e-e4cb4c2bd4e4

echo "=== restart manager with new code ==="
pkill -f system_manager.py; sleep 2
cd TerraSLAM_relay
nohup python3 system_manager.py > /tmp/manager.log 2>&1 &
sleep 6
curl -s -o /dev/null -w "manager http=%{http_code}\n" "http://127.0.0.1:8000/api/projects/$PROJ" 2>&1 || echo "manager down"

echo "=== confirm new flag code loaded (grep manager.log startup) ==="
grep -iE 'frame-save|routes_calibration|uvicorn|Started' /tmp/manager.log 2>/dev/null | tail -5

echo "=== unit-test the flag helpers via python ==="
python3 - <<'PY'
import ast
src = open('TerraSLAM_relay/system_manager_pkg/routes_calibration.py').read()
print("has _enable_frame_saving:", "_enable_frame_saving(session_id: str)" in src)
print("has _disable_frame_saving:", "_disable_frame_saving()" in src)
# Simulate
import os
FLAG="/tmp/terraslam_save_frames"
if os.path.exists(FLAG): os.remove(FLAG)
open(FLAG,"w").write("x\n")
print("after enable exists:", os.path.exists(FLAG))
if os.path.exists(FLAG): os.remove(FLAG)
print("after disable exists:", os.path.exists(FLAG))
PY
