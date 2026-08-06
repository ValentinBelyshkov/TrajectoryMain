#!/bin/bash
P=/opt/main/Trajectory/Database/projects/cc6494d4-2177-4812-b08e-e4cb4c2bd4e4
python3 - <<'PY'
import json
mp=f"{'/opt/main/Trajectory/Database/projects/cc6494d4-2177-4812-b08e-e4cb4c2bd4e4'}/metadata.json"
d=json.load(open(mp))
d["calibration_status"]="calibrated"
json.dump(d,open(mp,"w"),indent=2,ensure_ascii=False)
print("metadata calibration_status ->", d.get("calibration_status"))
PY
