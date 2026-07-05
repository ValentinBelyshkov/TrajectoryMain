"""
Component lifecycle routes: start/stop/restart, publisher mode/path,
relay calib path, and log retrieval/streaming.
"""
import asyncio
import os

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from typing import Optional

from . import process_manager as pm
from .config import COMPONENT_CONFIG, PUBLISHER_MODE_FILE, CalibPathReq, ModeReq, PathReq, ValueReq
from .process_manager import log_hub, manager

router = APIRouter()


@router.get("/health")
def health():
    return {"status": "ok", "manager": "running"}


@router.get("/api/v1/status")
def get_status():
    import time
    return {"timestamp": time.time(), "components": manager.get_status()}


@router.post("/api/v1/components/{component}/{action}")
async def component_action(component: str, action: str, req: Optional[ValueReq] = None):
    if action not in ("start", "stop", "restart", "kill"):
        raise HTTPException(status_code=400, detail=f"Invalid action: {action}")

    targets = []
    if component == "all":
        targets = list(COMPONENT_CONFIG.keys())
    elif component in COMPONENT_CONFIG:
        targets = [component]
    else:
        raise HTTPException(status_code=400, detail=f"Unknown component: {component}")

    results = []
    for comp in targets:
        try:
            if action == "kill":
                out = await manager.stop(comp, force=True)
            elif action == "stop":
                out = await manager.stop(comp)
            elif action == "start":
                extra = _extra_for(comp, req)
                out = await manager.start(comp, extra)
            elif action == "restart":
                extra = _extra_for(comp, req)
                out = await manager.restart(comp, extra)
            results.append(f"{comp}: {out}")
        except Exception as e:
            results.append(f"{comp}: error: {str(e)}")

    return {"success": True, "results": results}


def _extra_for(comp: str, req: Optional[ValueReq]):
    if comp == "publisher_folder":
        path = (req.value if req and req.value else None) or pm.publisher_folder_path
        if not path:
            raise ValueError("publisher_folder requires path — call POST /api/v1/publisher/path first")
        return {"path": path}
    if comp == "relay":
        if not pm.relay_calib_path:
            raise ValueError("relay calib_path not set — call POST /api/v1/relay/path first")
        return {"calib_path": pm.relay_calib_path}
    return None


@router.post("/api/v1/publisher/mode")
async def pub_mode(req: ModeReq):
    if req.mode not in ("folder", "realsense"):
        raise HTTPException(status_code=400, detail="Mode must be folder or realsense")

    with open(PUBLISHER_MODE_FILE, "w") as f:
        f.write(req.mode)

    for name in ("publisher_folder", "publisher_realsense"):
        try:
            await manager.stop(name)
        except Exception:
            pass

    await asyncio.sleep(1)

    target = f"publisher_{req.mode}"
    try:
        out = await manager.start(target)
        return {"success": True, "message": f"Switched to {req.mode}: {out}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/api/v1/relay/path")
async def relay_path(req: CalibPathReq):
    if not os.path.isfile(req.calib_path):
        raise HTTPException(status_code=400, detail=f"File does not exist: {req.calib_path}")
    pm.relay_calib_path = req.calib_path
    return {"success": True, "calib_path": req.calib_path}


@router.post("/api/v1/publisher/path")
async def pub_path(req: PathReq):
    if not os.path.isdir(req.path):
        raise HTTPException(status_code=400, detail=f"Directory does not exist: {req.path}")

    pm.publisher_folder_path = req.path
    print(f"[pub_path] Stored publisher path: {req.path}", flush=True)

    with open(PUBLISHER_MODE_FILE, "w") as f:
        f.write("folder")

    try:
        await manager.stop("publisher_folder")
    except Exception:
        pass

    await asyncio.sleep(0.5)

    try:
        out = await manager.start("publisher_folder", extra={"path": req.path})
        print(f"[pub_path] Started publisher_folder: {out}", flush=True)
        return {"success": True, "message": f"VIDEO_FOLDER updated and started: {out}"}
    except Exception as e:
        print(f"[pub_path] ERROR starting publisher_folder: {e}", flush=True)
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/api/v1/logs")
def get_logs(component: Optional[str] = None, limit: int = 100, since: Optional[float] = None):
    logs = log_hub.get_logs(component, limit, since)
    return {"count": len(logs), "logs": logs}


@router.websocket("/api/v1/logs/stream")
async def logs_ws(websocket: WebSocket):
    await websocket.accept()
    log_hub.subscribe(websocket)
    for entry in log_hub.get_logs(limit=50):
        await websocket.send_json(entry)
    try:
        while True:
            await asyncio.sleep(10)
    except WebSocketDisconnect:
        pass
    finally:
        log_hub.unsubscribe(websocket)
