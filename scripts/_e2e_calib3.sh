#!/bin/bash
cd /opt/main/Trajectory
PROJ=cc6494d4-2177-4812-b08e-e4cb4c2bd4e4
PF=/opt/main/Trajectory/Database/projects/$PROJ/procframe
SID=calib_20260806_184541_abf9de

echo "=== wait for run to finish (poll progress) ==="
for i in $(seq 1 60); do
  prog=$(curl -s "http://127.0.0.1:9000/api/v1/calibration/session/$SID/progress" 2>/dev/null | python3 -c "import sys,json;d=json.load(sys.stdin);print(d.get('step'),'|',d.get('frames_done'),'|',d.get('status'))" 2>/dev/null)
  flag=$( [ -e /tmp/terraslam_save_frames ] && echo YES || echo no )
  jpg=$(ls "$PF"/*.jpg 2>/dev/null | wc -l)
  echo "[t=$((i*3))s] flag=$flag jpg=$jpg :: $prog"
  case "$prog" in *done*|*error*) break;; esac
  sleep 3
done

echo "=== FINAL ==="
echo "flag: $([ -e /tmp/terraslam_save_frames ] && echo YES || echo NO)"
echo "procframe jpg=$(ls "$PF"/*.jpg 2>/dev/null | wc -l) txt=$(ls "$PF"/*.txt 2>/dev/null | wc -l)"
echo "=== manager frame-save log ==="
grep -iE 'frame-save' /tmp/manager.log | tail -10
echo "=== find recent slam logs ==="
ls -t /opt/main/Trajectory/TerraSLAM_relay/logs/*.log 2>/dev/null | head -3
