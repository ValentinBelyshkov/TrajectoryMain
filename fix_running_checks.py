import pathlib

p = pathlib.Path("/opt/main/Trajectory/TWA/backend/app/routers/control.py")
text = p.read_text()

# Fix all_running check to use "OK" instead of "RUNNING"
text = text.replace(
    'components_status.get(comp, "").upper() == "RUNNING"',
    'components_status.get(comp, "").upper() == "OK"'
)

# Fix orphaned_processes check to use "OK" instead of "RUNNING"
text = text.replace(
    'if comp_name not in main_components and state.upper() == "RUNNING":',
    'if comp_name not in main_components and state.upper() == "OK":'
)

p.write_text(text)
