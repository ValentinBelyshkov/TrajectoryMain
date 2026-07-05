"""
YAML config updates, log hub (in-memory + websocket fan-out), and the
asyncio-based process manager used to start/stop/watch component processes.
"""
import asyncio
import os
import shlex
import signal
import time
from collections import deque
from typing import Any, Dict, List, Optional

from fastapi import WebSocket

from .config import COMPONENT_CONFIG


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
        if name == "relay":
            calib_path = (extra or {}).get("calib_path", "")
            if not calib_path:
                raise ValueError("relay requires calib_path")
            return [
                "bash", "-lc",
                f"source /opt/ros/humble/setup.bash && "
                f"source /home/orb/colcon_ws/install/setup.bash && "
                f"cd /home/orb/TerraSLAM_relay && "
                f"exec python3 gps_client.py {shlex.quote(calib_path)}"
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

# Пути, которые передаются отдельным запросом до старта компонента
relay_calib_path: Optional[str] = None
publisher_folder_path: Optional[str] = None
