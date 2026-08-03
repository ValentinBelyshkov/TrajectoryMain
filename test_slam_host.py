import urllib.request
import json

data = json.dumps({
    "project_id": "99114589-0369-4d26-a26b-74ee3c53f6fe",
    "mode": "folder",
    "duration": 5
}).encode()

req = urllib.request.Request(
    "http://localhost:9000/slam/run",
    data=data,
    headers={"Content-Type": "application/json"},
    method="POST"
)

try:
    with urllib.request.urlopen(req, timeout=60) as resp:
        print(resp.read().decode())
except Exception as e:
    print(f"ERROR: {e}")
