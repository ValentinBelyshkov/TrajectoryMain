#!/bin/bash
cd /opt/main/Trajectory
PROJ=cc6494d4-2177-4812-b08e-e4cb4c2bd4e4

echo "=== manager health (port 9000) ==="
curl -s -o /dev/null -w "http=%{http_code}\n" "http://127.0.0.1:9000/api/projects/$PROJ"
curl -s "http://127.0.0.1:9000/api/projects/$PROJ" -o /tmp/proj.json
python3 -c "import json;d=json.load(open('/tmp/proj.json'));print('calibration_status =', d.get('calibration_status'))"

echo "=== unit-test flag helpers ==="
python3 - <<'PY'
import os
FLAG="/tmp/terraslam_save_frames"
if os.path.exists(FLAG): os.remove(FLAG)
open(FLAG,"w").write("x\n")
print("after enable exists:", os.path.exists(FLAG))
if os.path.exists(FLAG): os.remove(FLAG)
print("after disable exists:", os.path.exists(FLAG))
PY

echo "=== confirm _run_calibration_process wires the flag (grep) ==="
grep -n '_enable_frame_saving\|_disable_frame_saving' TerraSLAM_relay/system_manager_pkg/routes_calibration.py
