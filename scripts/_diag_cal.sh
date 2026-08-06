#!/bin/bash
P=/opt/main/Trajectory/Database/projects/cc6494d4-2177-4812-b08e-e4cb4c2bd4e4
echo "=== metadata.json calibration_status ==="
grep -o '"calibration_status"[^,}]*' "$P/metadata.json" 2>/dev/null || echo "NOT FOUND"
echo "=== 1.json first 200 chars ==="
head -c 200 "$P/calibrations/1.json" 2>/dev/null; echo
echo "=== API get_project ==="
curl -s "http://127.0.0.1:8000/api/projects/cc6494d4-2177-4812-b08e-e4cb4c2bd4e4" -o /tmp/proj.json
python3 -c "import json;d=json.load(open('/tmp/proj.json'));print('calibration_status =',d.get('calibration_status'))" 2>&1
echo "=== API calibration/status ==="
curl -s "http://127.0.0.1:8000/api/projects/cc6494d4-2177-4812-b08e-e4cb4c2bd4e4/calibration/status"
echo
