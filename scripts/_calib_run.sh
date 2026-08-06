#!/bin/bash
# Restart system_manager so new flag-gated code is live, then run a calibration
# sequence and watch the flag file appear/disappear automatically.
cd /opt/main/Trajectory
PROJ=cc6494d4-2177-4812-b08e-e4cb4c2bd4e4

echo "=== restart manager ==="
pkill -f system_manager.py; sleep 2
cd TerraSLAM_relay
nohup python3 system_manager.py > /tmp/manager.log 2>&1 &
sleep 5
curl -s -o /dev/null -w "manager http=%{http_code}\n" http://127.0.0.1:8000/api/status 2>&1 || echo "manager not up yet"

echo "=== find an existing calibration session to (re)run, or create one ==="
# Inspect existing 1.json session id if any
SID=$(python3 -c "import json;d=json.load(open('/opt/main/Trajectory/Database/projects/$PROJ/calibrations/1.json'));print(d.get('id'))" 2>/dev/null)
echo "session id = $SID"

echo "=== trigger calibration run via API (start) ==="
curl -s -X POST "http://127.0.0.1:8000/api/v1/calibration/run/$PROJ" -H 'Content-Type: application/json' -d '{"session_id":"'"$SID"'"}' 2>&1 | head -c 300; echo

echo "=== poll flag file for ~40s ==="
for i in $(seq 1 20); do
  if [ -e /tmp/terraslam_save_frames ]; then echo "[t=$((i*2))s] FLAG PRESENT"; else echo "[t=$((i*2))s] no flag"; fi
  sleep 2
done
echo "=== final flag state ==="
ls -la /tmp/terraslam_save_frames 2>&1 || echo "flag absent (calibration ended)"
