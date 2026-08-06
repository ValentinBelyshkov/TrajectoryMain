import asyncio
import json
import random
import os
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Dict, Set

router = APIRouter()

class TelemetryManager:
    def __init__(self):
        self.active_connections: Set[WebSocket] = set()
        self.project_telemetry: Dict[str, dict] = {}
        self._lock = asyncio.Lock()
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        async with self._lock:
            self.active_connections.add(websocket)
    
    async def disconnect(self, websocket: WebSocket):
        async with self._lock:
            self.active_connections.discard(websocket)
    
    async def broadcast(self, project_id: str, data: dict):
        message = json.dumps({"project_id": project_id, **data})
        async with self._lock:
            # Create a copy to avoid modification during iteration
            for connection in list(self.active_connections):
                try:
                    await connection.send_text(message)
                except Exception as e:
                    print(f"Broadcast error: {e}")


telemetry_manager = TelemetryManager()

@router.websocket("/ws/{project_id}")
async def telemetry_websocket(websocket: WebSocket, project_id: str):
    """
    WebSocket endpoint for real-time telemetry streaming.
    Sends: height, velocity (x,y,z), battery, gps_status, mode, status
    """
    await telemetry_manager.connect(websocket)
    
    base_lat = 55.7558
    base_lng = 37.6173
    
    try:
        while True:
            telemetry_data = {
                "height": round(random.uniform(1.0, 10.0), 1),
                "vx": round(random.uniform(-2.0, 2.0), 2),
                "vy": round(random.uniform(-2.0, 2.0), 2),
                "vz": round(random.uniform(-1.0, 1.0), 2),
                "battery": random.randint(50, 100),
                "gps_status": random.choice(["lock", "lock", "lock", "no_fix"]),
                "mode": random.choice(["manual", "auto", "hover"]),
                "status": "active",
                "lat": round(base_lat + random.uniform(-0.001, 0.001), 6),
                "lng": round(base_lng + random.uniform(-0.001, 0.001), 6),
                "timestamp": int(asyncio.get_event_loop().time() * 1000)
            }
            
            async with telemetry_manager._lock:
                await websocket.send_json(telemetry_data)
            await asyncio.sleep(0.5)
            
    except WebSocketDisconnect:
        await telemetry_manager.disconnect(websocket)

@router.post("/start")
async def start_telemetry(project_id: str):
    """Start telemetry streaming for a project."""
    return {"status": "started", "project_id": project_id}

@router.post("/stop")
async def stop_telemetry(project_id: str):
    """Stop telemetry streaming for a project."""
    return {"status": "stopped", "project_id": project_id}