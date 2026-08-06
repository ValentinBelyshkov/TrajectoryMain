#!/bin/bash
P=/opt/main/Trajectory/Database/projects/cc6494d4-2177-4812-b08e-e4cb4c2bd4e4
python3 - <<'PY'
import json
d=json.load(open("/opt/main/Trajectory/Database/projects/cc6494d4-2177-4812-b08e-e4cb4c2bd4e4/calibrations/1.json"))
for k in ["id","status","gpc_filename","calibration_file","project_id"]:
    print(k,"=",d.get(k))
PY
echo "=== gpc refs in frontend/backend ==="
cd /opt/main/Trajectory
grep -rn "calib.gpc\|\.gpc\|1\.json\|calibrations/1" TWA/client/ TWA/backend/ TerraSLAM_relay/ 2>/dev/null | grep -iv "node_modules" | head -30
