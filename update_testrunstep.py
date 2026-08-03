import pathlib

content = pathlib.Path(r"C:\Users\GlaDOS_laptop\Documents\Code\Trajectory\TestRunStep.tsx").read_text()
target = pathlib.Path("/opt/main/Trajectory/TWA/client/components/calibration/TestRunStep.tsx")
target.write_text(content)
print("TestRunStep.tsx updated")
