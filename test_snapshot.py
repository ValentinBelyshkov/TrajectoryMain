import urllib.request
import json

data = json.dumps({
    "path": "/opt/main/Trajectory/Database/projects/99114589-0369-4d26-a26b-74ee3c53f6fe/procframe/test.jpg"
}).encode()

req = urllib.request.Request(
    "http://localhost:9000/api/v1/camera/snapshot",
    data=data,
    headers={"Content-Type": "application/json"},
    method="POST"
)

try:
    with urllib.request.urlopen(req, timeout=30) as resp:
        print(resp.read().decode())
except urllib.error.HTTPError as e:
    print(f"HTTP Error: {e.code}")
    print(e.read().decode())
except Exception as e:
    print(f"ERROR: {e}")
