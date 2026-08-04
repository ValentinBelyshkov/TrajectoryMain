import re

files = [
    "/opt/main/Trajectory/TWA/client/pages/Index.tsx",
    "/opt/main/Trajectory/TWA/client/hooks/useProject.ts",
]

replacements = [
    (
        'const wsUrl = `${import.meta.env.VITE_WS_URL || "ws://localhost:8000"}/api/telemetry/ws/${project.id}`;',
        'const wsUrl = `${window.location.protocol === "https:" ? "wss:" : "ws:"}//${window.location.host}/api/telemetry/ws/${project.id}`;'
    ),
    (
        'const wsUrl = `${import.meta.env.VITE_WS_URL || "ws://localhost:9000"}/api/video/ws/${projectId}`;',
        'const wsUrl = `${window.location.protocol === "https:" ? "wss:" : "ws:"}//${window.location.host}/api/video/ws/${projectId}`;'
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
