#!/usr/bin/env python3
"""
TerraSLAM System Manager v1 — replaces Gateway + Supervisor.
Runs inside TerraSLAM container. Direct process control via asyncio.
"""
import asyncio
import glob
import os
import shlex
import signal
import time
from collections import deque
from typing import Optional, Dict, Any, List

import subprocess
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from pydantic import BaseModel
import uvicorn

# --- Config ---
DEFAULT_MODE = "folder"
PUBLISHER_MODE_FILE = "/tmp/terraslam_publisher_mode"

COMPONENT_CONFIG = {
    "slam": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && "
            "source /home/orb/colcon_ws/install/setup.bash && "
            "export PATH='/usr/bin:/bin:/opt/ros/humble/bin:$PATH' && "
            "export LD_LIBRARY_PATH='/home/orb/ORB_SLAM3/lib:/home/orb/colcon_ws/install/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH' && "
            "mkdir -p /home/orb/.ros/log && "
            "ros2 run orb_slam3_ros2_wrapper mono /home/orb/ORB_SLAM3/Vocabulary/ORBvoc.bin /home/orb/Database/real.yaml --ros-args -r __ns:=/"
        ],
        "cwd": "/home/orb",
        "env": {"ROS_DOMAIN_ID": "0", "QT_X11_NO_MITSHM": "1"},
        "autorestart": False,
        "max_restarts": 0,
    },
    "publisher_folder": {
        "cwd": "/home/orb/Database",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": False,
        "max_restarts": 0,
    },
    "publisher_realsense": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && source /home/orb/colcon_ws/install/setup.bash && exec python3 /home/orb/Database/realsense.py"
        ],
        "cwd": "/home/orb/Database",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": False,
        "max_restarts": 0,
    },
    "gps_bridge": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && source /home/orb/colcon_ws/install/setup.bash && "
            "python3 /home/orb/TerraSLAM_relay/Serial/gps_bridge_node.py --ros-args "
            "-p protocol:=msp -p hw_type:=uart -p port:=/dev/ttyCH341USB0 -p baudrate:=115200"
        ],
        "cwd": "/home/orb/TerraSLAM_relay/Serial",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": True,
        "max_restarts": 3,
        "restart_window": 60,
    },
    "rosbridge": {
        "cmd": [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && "
            "ros2 launch rosbridge_server rosbridge_websocket_launch.xml "
            "port:=9091 cors_origin:=* max_message_size:=10000000 fragment_timeout:=4000"
        ],
        "cwd": "/home/orb",
        "env": {"ROS_DOMAIN_ID": "0"},
        "autorestart": True,
        "max_restarts": 999,
        "restart_window": 60,
    },
    "slam_mode_manager": {
    "cmd": [
        "bash", "-lc",
        "source /opt/ros/humble/setup.bash && "
        "source /home/orb/colcon_ws/install/setup.bash && "
        "python3 /home/orb/TerraSLAM_relay/slam_mode_manager.py"
    ],
    "cwd": "/home/orb",
    "env": {"ROS_DOMAIN_ID": "0"},
    "autorestart": True,
    "max_restarts": 5,
    "restart_window": 60,
    }
}

# --- Pydantic Models ---
class ValueReq(BaseModel):
    value: Optional[str] = None

class ModeReq(BaseModel):
    mode: str

class PathReq(BaseModel):
    path: str

class RunReq(BaseModel):
    project_id: str
    mode: str = "folder"
    duration: int = 15


# --- YAML Updater ---
def update_yaml(yaml_path: str, save_path: str):
    """
    Updates ORB-SLAM3 YAML config using text replacement.
    Preserves OpenCV FileStorage format (%YAML:1.0 etc).
    """
    if os.path.exists(yaml_path):
        with open(yaml_path, "r") as f:
            lines = f.readlines()
    else:
        lines = []

    settings = {
        "System.SaveAtlas": "true",
        "System.LoadAtlas": "false",
        "System.AtlasLoadPath": '""',
        "System.AtlasSavePath": f'"{save_path}"',
    }

    updated = {}
    new_lines = []

    for line in lines:
        stripped = line.strip()
        replaced = False
        for key, value in settings.items():
            if stripped.startswith(f"{key}:") or stripped.startswith(f"{key} :"):
                indent = line[:len(line) - len(line.lstrip())]
                new_lines.append(f"{indent}{key}: {value}\n")
                updated[key] = True
                replaced = True
                break
        if not replaced:
            new_lines.append(line)

    missing = [k for k in settings if k not in updated]
    if missing:
        if new_lines and new_lines[-1].strip() != "":
            new_lines.append("\n")
        for key in missing:
            new_lines.append(f"{key}: {settings[key]}\n")

    os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    with open(yaml_path, "w") as f:
        f.writelines(new_lines)


# --- Log Hub ---
class LogHub:
    def __init__(self, maxlen: int = 5000):
        self._deque = deque(maxlen=maxlen)
        self._connections: set = set()

    async def push(self, component: str, source: str, level: str, message: str):
        entry = {
            "timestamp": time.time(),
            "component": component,
            "source": source,
            "level": level,
            "message": str(message)[:4096],
        }
        self._deque.append(entry)
        dead: set = set()
        for ws in self._connections:
            try:
                await ws.send_json(entry)
            except Exception:
                dead.add(ws)
        self._connections -= dead

    def get_logs(self, component: Optional[str] = None, limit: int = 100, since: Optional[float] = None) -> List[Dict[str, Any]]:
        logs = list(self._deque)
        if component and component != "all":
            logs = [l for l in logs if l.get("component") == component]
        if since:
            logs = [l for l in logs if l.get("timestamp", 0) > since]
        return logs[-limit:]

    def subscribe(self, ws: WebSocket):
        self._connections.add(ws)

    def unsubscribe(self, ws: WebSocket):
        self._connections.discard(ws)

log_hub = LogHub()


# --- Process Manager ---
class ProcessManager:
    def __init__(self):
        self._procs: Dict[str, Dict[str, Any]] = {}
        self._log_tasks: Dict[str, List[asyncio.Task]] = {}
        self._shutting_down = False

    def _build_cmd(self, name: str, extra: Optional[Dict[str, Any]] = None) -> List[str]:
        cfg = COMPONENT_CONFIG[name]
        if name == "publisher_folder":
            path = (extra or {}).get("path", "")
            if not path:
                raise ValueError("publisher_folder requires path")
            return [
                "bash", "-lc",
                f"source /opt/ros/humble/setup.bash && source /home/orb/colcon_ws/install/setup.bash && "
                f"cd /home/orb/Database && exec python3 image_publish.py {shlex.quote(path)}"
            ]
        return list(cfg["cmd"])

    async def start(self, name: str, extra: Optional[Dict[str, Any]] = None) -> str:
        if name in self._procs:
            proc = self._procs[name].get("proc")
            if proc and proc.returncode is None:
                return f"{name}: already running"

        cfg = COMPONENT_CONFIG.get(name)
        if not cfg:
            raise ValueError(f"Unknown component: {name}")

        cmd = self._build_cmd(name, extra)
        env = {**os.environ, **cfg.get("env", {})}

        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd,
                cwd=cfg.get("cwd"),
                env=env,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                start_new_session=True,
            )
        except Exception as e:
            raise RuntimeError(f"Failed to start {name}: {e}")

        now = time.time()
        self._procs[name] = {
            "proc": proc,
            "cmd": cmd,
            "start_time": now,
            "restarts": self._procs.get(name, {}).get("restarts", 0) + 1,
            "restart_times": self._procs.get(name, {}).get("restart_times", []) + [now],
            "autorestart": cfg.get("autorestart", False),
            "max_restarts": cfg.get("max_restarts", 0),
            "restart_window": cfg.get("restart_window", 60),
        }

        t1 = asyncio.create_task(self._pipe_logs(name, proc.stdout, "stdout"))
        t2 = asyncio.create_task(self._pipe_logs(name, proc.stderr, "stderr"))
        self._log_tasks[name] = [t1, t2]

        asyncio.create_task(self._watchdog(name))
        return f"{name}: started (pid {proc.pid})"

    async def stop(self, name: str, force: bool = False) -> str:
        info = self._procs.get(name)
        if not info or not info.get("proc"):
            return f"{name}: not running"

        proc: asyncio.subprocess.Process = info["proc"]
        if proc.returncode is not None:
            return f"{name}: already exited ({proc.returncode})"

        for t in self._log_tasks.pop(name, []):
            t.cancel()

        try:
            pgid = os.getpgid(proc.pid)
            if force:
                os.killpg(pgid, signal.SIGKILL)
            else:
                os.killpg(pgid, signal.SIGTERM)
                try:
                    await asyncio.wait_for(proc.wait(), timeout=5.0)
                except asyncio.TimeoutError:
                    os.killpg(pgid, signal.SIGKILL)
                    await proc.wait()
        except ProcessLookupError:
            pass

        try:
            if proc.stdout and not proc.stdout.at_eof():
                proc.stdout.close()
        except Exception:
            pass
        try:
            if proc.stderr and not proc.stderr.at_eof():
                proc.stderr.close()
        except Exception:
            pass

        return f"{name}: stopped"

    async def restart(self, name: str, extra: Optional[Dict[str, Any]] = None) -> str:
        await self.stop(name, force=True)
        await asyncio.sleep(1)
        return await self.start(name, extra)

    async def _pipe_logs(self, name: str, stream, source: str):
        try:
            while True:
                line = await stream.readline()
                if not line:
                    break
                text = line.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                level = self._detect_level(text)
                await log_hub.push(name, source, level, text)
        except asyncio.CancelledError:
            pass
        except Exception:
            pass

    def _detect_level(self, line: str) -> str:
        line_upper = line.upper()
        if "[ERROR]" in line_upper or "[FATAL]" in line_upper:
            return "ERROR"
        if "[WARN]" in line_upper:
            return "WARN"
        if "[DEBUG]" in line_upper:
            return "DEBUG"
        return "INFO"

    async def _watchdog(self, name: str):
        info = self._procs.get(name)
        if not info:
            return
        proc: asyncio.subprocess.Process = info["proc"]
        returncode = await proc.wait()

        info["proc"] = None
        self._log_tasks.pop(name, None)

        if self._shutting_down:
            return

        if not info["autorestart"]:
            await log_hub.push(name, "watchdog", "WARN", f"Exited ({returncode}), autorestart disabled")
            return

        window = info["restart_window"]
        max_restarts = info["max_restarts"]
        recent = [t for t in info.get("restart_times", []) if time.time() - t < window]
        if len(recent) > max_restarts:
            await log_hub.push(name, "watchdog", "ERROR", f"Too many restarts ({len(recent)} in {window}s)")
            return

        await log_hub.push(name, "watchdog", "WARN", f"Exited ({returncode}), restarting in 1s...")
        await asyncio.sleep(1)
        try:
            await self.start(name)
        except Exception as e:
            await log_hub.push(name, "watchdog", "ERROR", f"Restart failed: {e}")

    def get_status(self) -> List[Dict[str, Any]]:
        components = []
        for name in COMPONENT_CONFIG.keys():
            info = self._procs.get(name, {})
            proc = info.get("proc")
            running = proc is not None and proc.returncode is None

            if running:
                level = 0
                level_name = "OK"
                message = f"RUNNING (pid {proc.pid})"
            elif info.get("restarts", 0) > 0 and not self._shutting_down:
                level = 1
                level_name = "WARN"
                message = "STOPPED"
            else:
                level = 2
                level_name = "ERROR"
                message = "NOT RUNNING"

            components.append({
                "name": f"terraslam/{name}",
                "level": level,
                "level_name": level_name,
                "message": message,
                "values": {
                    "program": name,
                    "pid": proc.pid if running else None,
                    "returncode": proc.returncode if proc else None,
                    "restarts": info.get("restarts", 0),
                },
            })
        return components

    async def stop_all(self):
        self._shutting_down = True
        for name in list(self._procs.keys()):
            await self.stop(name, force=True)

manager = ProcessManager()


# --- FastAPI ---
app = FastAPI(title="TerraSLAM System Manager")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

@app.on_event("startup")
async def startup():
    try:
        res = await manager.start("rosbridge")
        print(f"[startup] {res}")
    except Exception as e:
        print(f"[startup] Failed to start rosbridge: {e}")

@app.on_event("shutdown")
async def shutdown():
    await manager.stop_all()

@app.get("/health")
def health():
    return {"status": "ok", "manager": "running"}

@app.get("/api/v1/status")
def get_status():
    return {"timestamp": time.time(), "components": manager.get_status()}

@app.post("/api/v1/components/{component}/{action}")
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
                extra = None
                if comp == "publisher_folder" and req and req.value:
                    extra = {"path": req.value}
                out = await manager.start(comp, extra)
            elif action == "restart":
                extra = None
                if comp == "publisher_folder" and req and req.value:
                    extra = {"path": req.value}
                out = await manager.restart(comp, extra)
            results.append(f"{comp}: {out}")
        except Exception as e:
            results.append(f"{comp}: error: {str(e)}")

    return {"success": True, "results": results}

@app.post("/api/v1/publisher/mode")
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

@app.post("/api/v1/publisher/path")
async def pub_path(req: PathReq):
    if not os.path.isdir(req.path):
        raise HTTPException(status_code=400, detail=f"Directory does not exist: {req.path}")

    with open(PUBLISHER_MODE_FILE, "w") as f:
        f.write("folder")

    try:
        await manager.stop("publisher_folder")
    except Exception:
        pass

    await asyncio.sleep(1)

    try:
        out = await manager.start("publisher_folder", extra={"path": req.path})
        return {"success": True, "message": f"VIDEO_FOLDER updated and started: {out}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/v1/logs")
def get_logs(
    component: Optional[str] = None,
    limit: int = 100,
    since: Optional[float] = None
):
    logs = log_hub.get_logs(component, limit, since)
    return {"count": len(logs), "logs": logs}

@app.websocket("/api/v1/logs/stream")
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

@app.post("/slam/run")
async def slam_run(req: RunReq):
    frames_dir = f"/home/orb/Database/projects/{req.project_id}/frames"
    if req.mode == "folder" and not os.path.isdir(frames_dir):
        raise HTTPException(status_code=400, detail=f"Frames dir missing: {frames_dir}")

    results = []
    yaml_path = "/home/orb/Database/real.yaml"
    save_path = f"/home/orb/Database/projects/{req.project_id}/calibrations/map"

    try:
        update_yaml(yaml_path, save_path)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"YAML update failed: {e}")

    # 1. Start publisher
    publisher_name = f"publisher_{req.mode}"
    try:
        if req.mode == "folder":
            out = await manager.start("publisher_folder", extra={"path": frames_dir})
        else:
            out = await manager.start("publisher_realsense")
        results.append(f"publisher: {out}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Publisher failed: {e}")

    await asyncio.sleep(2)

    # 2. Start SLAM
    try:
        out = await manager.start("slam")
        results.append(f"slam: {out}")
    except Exception as e:
        # Cleanup publisher on SLAM failure
        try:
            await manager.stop(publisher_name)
        except Exception:
            pass
        raise HTTPException(status_code=500, detail=f"SLAM failed: {e}")

    # 3. Wait
    await asyncio.sleep(req.duration)

    # 4. Stop publisher first, then slam
    try:
        out = await manager.stop(publisher_name)
        results.append(f"publisher stop: {out}")
    except Exception as e:
        results.append(f"publisher stop error: {e}")

    await asyncio.sleep(2)

    try:
        out = await manager.stop("slam")
        results.append(f"slam stop: {out}")
    except Exception as e:
        results.append(f"slam stop error: {e}")

    # 5. Check for .osa result
    calib_dir = f"/home/orb/Database/projects/{req.project_id}/calibrations"
    expected_osa = os.path.join(calib_dir, "map.osa")
    osa_found = None

    if os.path.exists(expected_osa):
        osa_found = expected_osa
    else:
        try:
            recent = sorted(
                glob.glob(os.path.join(calib_dir, "*.osa")),
                key=os.path.getmtime,
                reverse=True
            )
            if recent:
                osa_found = recent[0]
        except Exception:
            pass

    return {
        "success": True,
        "results": results,
        "osa_file": osa_found,
        "project_id": req.project_id,
    }


# ========== SLAM MODE CONTROL ENDPOINTS ==========

# Глобальная переменная для хранения последнего статуса SLAM
_slam_status = {
    "slam_state": -1,
    "slam_state_name": "UNKNOWN",
    "drone_mode": "LOCALIZATION_ONLY",
    "initialized": False,
    "timestamp": 0
}

@app.get("/api/v1/slam/status")
def get_slam_status():
    """Получить текущий статус SLAM (который приходит из ROS2 по /slam/mode_status)"""
    return {
        "success": True,
        "status": _slam_status,
        "manager_time": time.time()
    }

@app.post("/api/v1/slam/mode")
async def set_slam_mode(req: ModeReq):
    """
    Переключить режим SLAM.
    mode: "LOCALIZATION_ONLY" (команда 3) или "SLAM_MAPPING" (команда 4)
    """
    valid_modes = ["LOCALIZATION_ONLY", "SLAM_MAPPING"]
    if req.mode not in valid_modes:
        raise HTTPException(400, detail=f"Mode must be one of: {valid_modes}")
    
    # Публикуем в ROS2 топик /slam/mode_command
    # Этот топик читает slam_mode_manager (ROS2 нод), который вызывает MapControl сервис
    try:
        # Пишем команду в файл-флаг (slam_mode_manager будет читать)
        mode_file = "/tmp/terraslam_slam_mode"
        with open(mode_file, "w") as f:
            f.write(req.mode)
        
        return {
            "success": True,
            "mode": req.mode,
            "message": f"Mode command written to {mode_file}. slam_mode_manager will process it."
        }
    except Exception as e:
        raise HTTPException(500, detail=f"Failed to set mode: {e}")

@app.post("/api/v1/slam/command")
async def slam_command(req: ValueReq):
    """
    Прямые команды SLAM:
    - "reset" (команда 0)
    - "save_map" (команда 1) — требует filepath в req.value
    - "load_map" (команда 2) — требует filepath в req.value
    """
    if not req.value:
        raise HTTPException(400, detail="value required: reset | save_map | save_map|/path | load_map|/path")

    # Support "cmd|/optional/path" format sent from the dashboard
    parts = req.value.split("|", 1)
    cmd = parts[0].strip().lower()
    filepath = parts[1].strip() if len(parts) > 1 else "/home/orb/Database/map.osa"

    valid = ["reset", "save_map", "load_map"]
    if cmd not in valid:
        raise HTTPException(400, detail=f"Command must be one of: {valid}")

    try:
        cmd_file = "/tmp/terraslam_slam_cmd"
        with open(cmd_file, "w") as f:
            f.write(f"{cmd}\n")

        if cmd in ["save_map", "load_map"]:
            path_file = "/tmp/terraslam_slam_path"
            with open(path_file, "w") as f:
                f.write(filepath)

        return {
            "success": True,
            "command": cmd,
            "filepath": filepath if cmd != "reset" else None,
            "message": f"Command '{cmd}' written to {cmd_file}",
        }
    except Exception as e:
        raise HTTPException(500, detail=f"Failed to write command: {e}")
        



# ═══════════════════════════════════════════════════════════════════════════════
#  DASHBOARD  — serve HTML at GET /
# ═══════════════════════════════════════════════════════════════════════════════

_DASHBOARD_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dashboard.html")

@app.get("/", response_class=HTMLResponse)
def dashboard():
    try:
        with open(_DASHBOARD_PATH, "r", encoding="utf-8") as f:
            return HTMLResponse(content=f.read())
    except FileNotFoundError:
        return HTMLResponse(content="<h1>dashboard.html not found</h1>", status_code=404)


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS2 DIRECT COMMANDS  /api/v1/ros/…
# ═══════════════════════════════════════════════════════════════════════════════

ROS2_TOPICS = {
    "covariance":   "/orb_slam3/covariance",
    "camera_pose":  "/orb_slam3/camera_pose",
    "imu":          "/imu/data",
    "mavros_state": "/mavros/state",
    "mavros_imu":   "/mavros/imu/data",
    "mavros_pose":  "/mavros/local_position/pose",
    "tracking":     "/orb_slam3/tracking_state",
}


def _ros2_run(ros2_args: list, timeout: int = 5):
    cmd = ["ros2"] + ros2_args
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout,
            env={**os.environ, "ROS_DOMAIN_ID": "0"},
        )
        return proc.stdout.strip(), proc.stderr.strip(), proc.returncode
    except FileNotFoundError:
        return "", "ros2 not found — run inside the Docker container", 127
    except subprocess.TimeoutExpired:
        return "", f"Timeout after {timeout}s", 1
    except Exception as e:
        return "", str(e), 1


@app.get("/api/v1/ros/topics")
def ros2_topic_list():
    stdout, stderr, rc = _ros2_run(["topic", "list"])
    topics = [t.strip() for t in stdout.splitlines() if t.strip()] if rc == 0 else []
    return {"topics": topics, "error": stderr, "returncode": rc}


@app.get("/api/v1/ros/nodes")
def ros2_node_list():
    stdout, stderr, rc = _ros2_run(["node", "list"])
    nodes = [n.strip() for n in stdout.splitlines() if n.strip()] if rc == 0 else []
    return {"nodes": nodes, "error": stderr, "returncode": rc}


@app.get("/api/v1/ros/topic/{name}")
def ros2_topic_echo(name: str):
    topic = ROS2_TOPICS.get(name)
    if not topic:
        raise HTTPException(400, detail=f"Unknown topic '{name}'. Known: {list(ROS2_TOPICS.keys())}")
    stdout, stderr, rc = _ros2_run(["topic", "echo", "--once", topic], timeout=5)
    return {"topic": topic, "output": stdout, "error": stderr, "returncode": rc}


class ArmReq(BaseModel):
    arm: bool = True

class FlightModeReq(BaseModel):
    mode: str


@app.post("/api/v1/ros/mavros/arm")
def ros2_mavros_arm(req: ArmReq):
    val = "true" if req.arm else "false"
    stdout, stderr, rc = _ros2_run(
        ["service", "call", "/mavros/cmd/arming",
         "mavros_msgs/srv/CommandBool", f"{{value: {val}}}"], timeout=8
    )
    return {"output": stdout, "error": stderr, "returncode": rc}


@app.post("/api/v1/ros/mavros/set_mode")
def ros2_mavros_set_mode(req: FlightModeReq):
    stdout, stderr, rc = _ros2_run(
        ["service", "call", "/mavros/set_mode",
         "mavros_msgs/srv/SetMode", f'{{custom_mode: "{req.mode}"}}'], timeout=8
    )
    return {"output": stdout, "error": stderr, "returncode": rc}


if __name__ == "__main__":
    port = int(os.environ.get("TERRASLAM_PORT", "5000"))
    uvicorn.run(app, host="0.0.0.0", port=port)

