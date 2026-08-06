#!/bin/bash
cd /opt/main/Trajectory
PROJ=cc6494d4-2177-4812-b08e-e4cb4c2bd4e4
PF=/opt/main/Trajectory/Database/projects/$PROJ/procframe
SID=calib_20260806_184541_abf9de

# Reset session to a processable state (as the UI would before processing)
python3 -c "
import json
p='/opt/main/Trajectory/Database/projects/$PROJ/calibrations/1.json'
d=json.load(open(p))
d['status']='recording'
d.pop('frame_pose_data',None)
json.dump(d,open(p,'w'),indent=2)
print('reset status ->', d['status'])
"

rm -f "$PF"/*.jpg "$PF"/*.txt 2>/dev/null

echo "=== POST /process ==="
curl -s -X POST "http://127.0.0.1:9000/api/v1/calibration/process" \
  -H 'Content-Type: application/json' \
  -d "{\"session_id\":\"$SID\",\"project_id\":\"$PROJ\"}"; echo

echo "=== watch flag + procframe ==="
for i in $(seq 1 40); do
  flag=$( [ -e /tmp/terraslam_save_frames ] && echo YES || echo no )
  jpg=$(ls "$PF"/*.jpg 2>/dev/null | wc -l)
  prog=$(curl -s "http://127.0.0.1:9000/api/v1/calibration/session/$SID/progress" 2>/dev/null | python3 -c "import sys,json;d=json.load(sys.stdin);print(d.get('step'),'| frames_done=',d.get('frames_done'),'| status=',d.get('status'))" 2>/dev/null)
  echo "[t=$((i*2))s] flag=$flag jpg=$jpg :: $prog"
  sleep 2
done
echo "=== FINAL ==="
echo "flag present? $([ -e /tmp/terraslam_save_frames ] && echo YES || echo NO)"
echo "procframe jpg=$(ls "$PF"/*.jpg 2>/dev/null | wc -l) txt=$(ls "$PF"/*.txt 2>/dev/null | wc -l)"

