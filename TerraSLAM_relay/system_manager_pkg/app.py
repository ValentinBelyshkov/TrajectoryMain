"""
TerraSLAM System Manager — FastAPI app assembly.
Replaces Gateway + Supervisor. Runs inside the TerraSLAM container.

Split into focused modules (~400 lines each):
- config.py               component definitions + request/response models
- process_manager.py      LogHub + asyncio ProcessManager + YAML updater
- pose_monitor.py         pose-topic liveness + tracking-lost detection (background poller)
- fallback_controller.py  configurable PosHold/RTL/Land fallback sent over MAVLink
- hardware_calibration.py MAVLink EKF3/ExternalNav calibration on real hardware
- routes_components.py    start/stop/restart, publisher/relay paths, logs
- routes_slam.py          slam run/mode/status/command, pose check, hw calibrate, fallback get/set
- routes_ros.py           dashboard, raw ROS2 CLI passthrough, compat aliases
- ws_routes.py            WebSocket push of live pose/fallback status
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from . import routes_components, routes_ros, routes_slam, ws_routes
from .pose_monitor import pose_monitor
from .process_manager import manager

app = FastAPI(title="TerraSLAM System Manager")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

app.include_router(routes_components.router)
app.include_router(routes_slam.router)
app.include_router(routes_ros.router)
app.include_router(ws_routes.router)


@app.on_event("startup")
async def startup():
    try:
        res = await manager.start("rosbridge")
        print(f"[startup] {res}")
    except Exception as e:
        print(f"[startup] Failed to start rosbridge: {e}")
    pose_monitor.start()


@app.on_event("shutdown")
async def shutdown():
    await pose_monitor.stop()
    await manager.stop_all()
