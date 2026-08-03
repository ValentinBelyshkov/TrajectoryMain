import pathlib

p = pathlib.Path("/opt/main/Trajectory/TWA/backend/app/routers/control.py")
text = p.read_text()
text = text.replace(
    'components_status[short_name] = comp.get("message", "UNKNOWN")',
    'components_status[short_name] = comp.get("level_name", "UNKNOWN")'
)
p.write_text(text)
