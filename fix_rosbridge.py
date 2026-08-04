import re

files = [
    "/opt/main/Trajectory/TWA/client/hooks/useProject.ts",
]

replacements = [
    (
        'const gpsUrl = `ws://${rosbridgeHost || "localhost"}:${rosbridgePort || "9091"}`;',
        'const gpsUrl = `ws://${rosbridgeHost || window.location.hostname}:${rosbridgePort || "9091"}`;'
    ),
]

for filepath in files:
    with open(filepath, "r", encoding="utf-8") as f:
        content = f.read()
    for old, new in replacements:
        content = content.replace(old, new)
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"Fixed: {filepath}")
