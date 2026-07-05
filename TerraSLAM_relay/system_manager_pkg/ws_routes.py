"""
WebSocket endpoint for live pose/fallback status push.

Instead of the dashboard polling /api/v1/slam/pose_status on an interval,
clients can open a WebSocket and receive pose_monitor.status() pushed on
every change (and at least every `WS_HEARTBEAT_SECONDS` as a heartbeat),
so pose loss / tracking-lost / fallback events show up in near real time.
"""
import asyncio
import json

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from .pose_monitor import pose_monitor

router = APIRouter()

WS_PUSH_INTERVAL_SECONDS = 1.0


@router.websocket("/ws/pose_status")
async def ws_pose_status(websocket: WebSocket):
    """Streams pose/fallback status as JSON, once per WS_PUSH_INTERVAL_SECONDS,
    for as long as the client stays connected."""
    await websocket.accept()
    last_sent = None
    try:
        while True:
            status = pose_monitor.status()
            if status != last_sent:
                await websocket.send_json({"type": "pose_status", "data": status})
                last_sent = status
            else:
                await websocket.send_json({"type": "heartbeat", "data": status})
            await asyncio.sleep(WS_PUSH_INTERVAL_SECONDS)
    except WebSocketDisconnect:
        pass
    except Exception as e:
        try:
            await websocket.send_json({"type": "error", "error": str(e)})
        except Exception:
            pass


@router.websocket("/ws/fallback_events")
async def ws_fallback_events(websocket: WebSocket):
    """Streams a message only when a new fallback event fires (tracking
    lost / pose stale), rather than the full periodic status feed."""
    await websocket.accept()
    last_trigger_ts = None
    try:
        while True:
            trigger = pose_monitor.status().get("last_fallback")
            ts = trigger.get("timestamp") if trigger else None
            if ts is not None and ts != last_trigger_ts:
                last_trigger_ts = ts
                await websocket.send_json({"type": "fallback_event", "data": trigger})
            await asyncio.sleep(WS_PUSH_INTERVAL_SECONDS)
    except WebSocketDisconnect:
        pass
    except Exception as e:
        try:
            await websocket.send_json({"type": "error", "error": str(e)})
        except Exception:
            pass
