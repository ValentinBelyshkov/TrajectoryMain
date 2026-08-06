#!/bin/bash
cd /opt/main/Trajectory
PROJ=cc6494d4-2177-4812-b08e-e4cb4c2bd4e4
PF=/opt/main/Trajectory/Database/projects/$PROJ/procframe

# get session id from 1.json
SID=$(python3 -c "import json;d=json.load(open('/opt/main/Trajectory/Database/projects/$PROJ/calibrations/1.json'));print(d.get('id'))")
echo "session id = $SID  status = $(python3 -c "import json;d=json.load(open('/opt/main/Trajectory/Database/projects/$PROJ/calibrations/1.json'));print(d.get('status'))")"

# clear procframe so we can see fresh saves
rm -f "$PF"/*.jpg "$PF"/*.txt 2>/dev/null

echo "=== POST /process ==="
curl -s -X POST "http://127.0.0.1:9000/api/v1/calibration/process" \
  -H 'Content-Type: application/json' \
  -d "{\"session_id\":\"$SID\",\"project_id\":\"$PROJ\"}" | head -c 300; echo

echo "=== watch flag + procframe for 60s ==="
for i in $(seq 1 30); do
  flag=$( [ -e /tmp/terraslam_save_frames ] && echo YES || echo no )
  jpg=$(ls "$PF"/*.jpg 2>/dev/null | wc -l)
  prog=$(curl -s "http://127.0.0.1:9000/api/v1/calibration/session/$SID/progress" 2>/dev/null | python3 -c "import sys,json;d=json.load(sys.stdin);print(d.get('step'),d.get('frames_done'),d.get('status'))" 2>/dev/null)
  echo "[t=$((i*2))s] flag=$flag procframe_jpg=$jpg progress=$prog"
  sleep 2
done

echo "=== FINAL ==="
echo "flag present? $([ -e /tmp/terraslam_save_frames ] && echo YES || echo NO)"
echo "procframe jpg = $(ls "$PF"/*.jpg 2>/dev/null | wc -l)"
echo "procframe txt = $(ls "$PF"/*.txt 2>/dev/null | wc -l)"
