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
from fastapi.staticfiles import StaticFiles
from fastapi import Request
from fastapi.responses import FileResponse, JSONResponse, Response
import os

from . import routes_components, routes_ros, routes_slam, routes_gps, ws_routes
from .pose_monitor import pose_monitor
from .process_manager import manager
from . import camera_stream

app = FastAPI(title="TerraSLAM System Manager")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

app.include_router(routes_components.router)
app.include_router(routes_slam.router)
app.include_router(routes_ros.router)
app.include_router(routes_gps.router)
app.include_router(ws_routes.router)

app.mount("/assets", StaticFiles(directory="/opt/main/Trajectory/TerraSLAM_relay/assets"), name="assets")

_TILES_ROOT = "/opt/main/Maps/tiles"
os.makedirs(_TILES_ROOT, exist_ok=True)


def _iter_candidate_paths(z: int, x: int, y: int):
    n = 1 << z
    y_xyz = y if 0 <= y < n else -1
    y_tms = n - 1 - y_xyz
    base = os.path.join(_TILES_ROOT, str(z), str(x))
    yield os.path.join(base, f"{y_tms}.pbf")
    yield os.path.join(base, f"{y_xyz}.pbf")
    yield os.path.join(base, f"{y}.pbf")


@app.get("/api/v1/map/tiles/{z}/{x}/{y}.pbf")
async def serve_tile(z: int, x: int, y: int, request: Request):
    path = None
    for candidate in _iter_candidate_paths(z, x, y):
        if os.path.isfile(candidate):
            path = candidate
            break
    if path is None:
        return JSONResponse(content={"detail": "tile not found"}, status_code=404)

    data = open(path, "rb").read()
    headers = {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "GET, OPTIONS",
        "Access-Control-Allow-Headers": "*",
        "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
    }
    headers["Content-Type"] = "application/vnd.mapbox-vector-tile"
    if path.endswith(".pbf.gz") or data[:2] == bytes([0x1F, 0x8B]):
        headers["Content-Encoding"] = "gzip"

    return Response(content=data, headers=headers)


@app.options("/api/v1/map/tiles/{z}/{x}/{y}.pbf")
async def tile_options(z: int, x: int, y: int):
    return JSONResponse(
        content={},
        headers={
            "Access-Control-Allow-Origin": "*",
            "Access-Control-Allow-Methods": "GET, OPTIONS",
            "Access-Control-Allow-Headers": "*",
        },
    )


@app.get("/tiles/{z}/{x}/{y}.pbf")
async def serve_tile_alias(z: int, x: int, y: int, request: Request):
    return await serve_tile(z, x, y, request)


@app.on_event("startup")
async def startup():
    try:
        res = await manager.start("rosbridge")
        print(f"[startup] {res}")
    except Exception as e:
        print(f"[startup] Failed to start rosbridge: {e}")
    pose_monitor.start()
    camera_stream.start()


@app.on_event("shutdown")
async def shutdown():
    await pose_monitor.stop()
    await manager.stop_all()
