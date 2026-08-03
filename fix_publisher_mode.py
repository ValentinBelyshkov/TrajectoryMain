import pathlib

p = pathlib.Path("/opt/main/Trajectory/TWA/backend/app/routers/control.py")
text = p.read_text()

# Fix publisher_mode detection to use "OK" instead of "RUNNING"
text = text.replace(
    'if components_status.get("publisher_folder", "").upper() == "RUNNING":',
    'if components_status.get("publisher_folder", "").upper() == "OK":'
)
text = text.replace(
    'elif components_status.get("publisher_realsense", "").upper() == "RUNNING":',
    'elif components_status.get("publisher_realsense", "").upper() == "OK":'
)

p.write_text(text)
