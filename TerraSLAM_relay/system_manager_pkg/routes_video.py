import asyncio
import json
import os
import base64
import websockets
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Set

router = APIRouter()

class VideoStreamManager:
    def __init__(self):
        self.active_connections: Set[WebSocket] = set()
        self.current_frame: str = ""
    
    async def connect(self, websocket: WebSocket):
        self.active_connections.add(websocket)
        print(f"Client connected. Total: {len(self.active_connections)}")
        if self.current_frame:
            try:
                await websocket.send_text(json.dumps({"type": "frame", "data": self.current_frame}))
            except:
                pass
    
    def disconnect(self, websocket: WebSocket):
        self.active_connections.discard(websocket)
        print(f"Client disconnected. Total: {len(self.active_connections)}")
    
    def has_active_connections(self) -> bool:
        """Check if there are any active WebSocket connections."""
        return len(self.active_connections) > 0
    
    async def broadcast(self, frame_data: str):
        if not self.has_active_connections():
            return
            
        self.current_frame = frame_data
        message = json.dumps({"type": "frame", "data": frame_data})
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                print(f"Send error: {e}")
                disconnected.append(connection)
        
        for conn in disconnected:
            self.active_connections.discard(conn)

video_stream_manager = VideoStreamManager()

ROSBRIDGE_HOST = os.getenv("ROSBRIDGE_HOST", "localhost")
ROSBRIDGE_PORT = int(os.getenv("ROSBRIDGE_PORT", 9091))

async def connect_to_rosbridge():
    """Connect to rosbridge WebSocket and subscribe to /camera/image_raw/compressed."""
    ws_url = f"ws://{ROSBRIDGE_HOST}:{ROSBRIDGE_PORT}"
    
    while True:
        try:
            print(f"Connecting to rosbridge at {ws_url}...")
            async with websockets.connect(ws_url, max_size=10_000_000) as websocket:
                print(f"Connected to rosbridge!")
                
                subscribe_msg = {
                    "op": "subscribe",
                    "topic": "/camera/image_raw/compressed",
                    "type": "sensor_msgs/msg/CompressedImage",
                    "queue_length": 1
                }
                await websocket.send(json.dumps(subscribe_msg))
                print(f"Subscription sent for /camera/image_raw/compressed")
                
                async for message in websocket:
                    try:
                        msg = json.loads(message)
                        
                        if msg.get("op") == "fragment":
                            continue
                        
                        if msg.get("topic") == "/camera/image_raw/compressed" and "msg" in msg:
                            img_data = msg["msg"].get("data")
                            
                            if img_data:
                                if isinstance(img_data, list):
                                    img_bytes = bytes(img_data)
                                    img_b64 = base64.b64encode(img_bytes).decode('utf-8')
                                    await video_stream_manager.broadcast(img_b64)
                                elif isinstance(img_data, str):
                                    await video_stream_manager.broadcast(img_data)
                                else:
                                    print(f"Unknown data type: {type(img_data)}")
                            else:
                                print("No image data")
                                
                    except Exception as e:
                        print(f"Message processing error: {e}")
                        import traceback
                        traceback.print_exc()
                        continue
                        
        except Exception as e:
            print(f"Rosbridge error: {e}")
            import traceback
            traceback.print_exc()
        
        await asyncio.sleep(3)

video_task = None

@router.websocket("/ws/{project_id}")
async def video_websocket(websocket: WebSocket, project_id: str):
    global video_task
    
    await websocket.accept()
    print(f"WebSocket accepted for project {project_id}")
    
    try:
        await video_stream_manager.connect(websocket)
        
        if video_task is None or video_task.done():
            video_task = asyncio.create_task(connect_to_rosbridge())
        
        while True:
            data = await websocket.receive_text()
            
    except WebSocketDisconnect:
        print(f"Client disconnected from project {project_id}")
        video_stream_manager.disconnect(websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        video_stream_manager.disconnect(websocket)
