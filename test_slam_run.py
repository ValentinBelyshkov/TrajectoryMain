import urllib.request
import json

data = json.dumps({"project_id": "test", "mode": "folder", "duration": 5}).encode()
req = urllib.request.Request(
    "http://host.docker.internal:9000/slam/run",
    data=data,
    headers={"Content-Type": "application/json"},
    method="POST"
)
try:
    with urllib.request.urlopen(req, timeout=60) as resp:
        print(resp.read().decode())
except Exception as e:
    print(f"ERROR: {e}")
