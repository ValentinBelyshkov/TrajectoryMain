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
from .state_store import state_store, atomic_write


# YAML updater moved to utils/yaml_editor.py
from .utils.yaml_editor import update_yaml

# Publishers are mutually exclusive — only one may run at a time.
PUBLISHER_GROUP = ("publisher_folder", "publisher_realsense")


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
        self._state = state_store
        self._projects_path = os.getenv("PROJECTS_DIR", "/opt/main/Trajectory/Database/projects")

    # --- desired-state / persistence ---
    def ensure_state(self, projects_path: Optional[str] = None):
        if projects_path:
            self._projects_path = str(projects_path)
        if not self._state._loaded:
            self._state.init(self._projects_path)

    def _publisher_sibling(self, name: str) -> Optional[str]:
        if name == "publisher_folder":
            return "publisher_realsense"
        if name == "publisher_realsense":
            return "publisher_folder"
        return None

    async def _enforce_exclusivity(self, name: str):
        """Stop the sibling publisher if running and clear its desired flag."""
        sibling = self._publisher_sibling(name)
        if not sibling:
            return
        info = self._procs.get(sibling)
        if info and info.get("proc") and info["proc"].returncode is None:
            await self.stop(sibling)
        self._state.set_desired(sibling, False)

    def set_autostart(self, name: str, enabled: bool) -> None:
        self.ensure_state()
        self._state.set_autostart(name, enabled)

    def _build_cmd(self, name: str, extra: Optional[Dict[str, Any]] = None) -> List[str]:
        cfg = COMPONENT_CONFIG[name]
        if name == "publisher_folder":
            path = (extra or {}).get("path", "")
            if not path:
                raise ValueError("publisher_folder requires path")
            procframe_dir = (extra or {}).get("procframe_dir")
            cmd = (
                "source /opt/ros/humble/setup.bash && "
                f"cd /opt/main/Trajectory/Database && exec python3 -u image_publish.py {shlex.quote(path)}"
            )
            if procframe_dir:
                cmd += f" --procframe-dir {shlex.quote(procframe_dir)}"
            if (extra or {}).get("once"):
                cmd += " --once"
            return ["bash", "-lc", cmd]
        if name == "publisher_realsense":
            frames_dir = (extra or {}).get("frames_dir")
            save_frames = (extra or {}).get("save_frames", True)
            cmd = (
                "source /opt/ros/humble/setup.bash && "
                "source /opt/main/Trajectory/host_colcon_ws/install/setup.bash && "
                "exec python3 /opt/main/Trajectory/Database/realsense.py"
            )
            if frames_dir:
                cmd += f" --frames-dir {shlex.quote(frames_dir)}"
            if not save_frames:
                cmd += " --no-save-frames"
            return ["bash", "-lc", cmd]
        return list(cfg["cmd"])

    async def start(self, name: str, extra: Optional[Dict[str, Any]] = None) -> str:
        self.ensure_state()

        if name in self._procs:
            proc = self._procs[name].get("proc")
            if proc and proc.returncode is None:
                self._state.set_desired(name, True, extra or {})
                return f"{name}: already running"

        cfg = COMPONENT_CONFIG.get(name)
        if not cfg:
            raise ValueError(f"Unknown component: {name}")

        # Publishers are mutually exclusive.
        if name in PUBLISHER_GROUP:
            await self._enforce_exclusivity(name)
            self._state.set_publisher_mode(
                "folder" if name == "publisher_folder" else "realsense"
            )

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
        prev = self._procs.get(name, {})
        self._procs[name] = {
            "proc": proc,
            "cmd": cmd,
            "start_time": now,
            "restarts": prev.get("restarts", 0),
            "restart_times": prev.get("restart_times", []),
            "autorestart": cfg.get("autorestart", False),
            "max_restarts": cfg.get("max_restarts", 0),
            "restart_window": cfg.get("restart_window", 60),
        }

        # Record that this component is now desired to be running.
        self._state.set_desired(name, True, extra or {})

        t1 = asyncio.create_task(self._pipe_logs(name, proc.stdout, "stdout"))
        t2 = asyncio.create_task(self._pipe_logs(name, proc.stderr, "stderr"))
        self._log_tasks[name] = [t1, t2]

        asyncio.create_task(self._watchdog(name))
        return f"{name}: started (pid {proc.pid})"

    async def stop(self, name: str, force: bool = False, clear_desired: bool = True) -> str:
        self.ensure_state()
        info = self._procs.get(name)
        if not info or not info.get("proc"):
            if clear_desired:
                self._state.set_desired(name, False)
            return f"{name}: not running"

        proc: asyncio.subprocess.Process = info["proc"]
        if proc.returncode is not None:
            if clear_desired:
                self._state.set_desired(name, False)
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

        # A normal stop means the component is no longer desired (so it won't be
        # auto-restarted or recovered on a manager restart). restart() and
        # stop_all() pass clear_desired=False to preserve desired-state.
        if clear_desired:
            self._state.set_desired(name, False)

        return f"{name}: stopped"

    async def restart(self, name: str, extra: Optional[Dict[str, Any]] = None) -> str:
        await self.stop(name, force=True, clear_desired=False)
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

        # Only auto-restart components that are *desired* to be running. Crash
        # recovery is now driven by desired-state, not the old `autorestart`
        # flag, so a manually stopped component is left stopped.
        desired = self._state.get_component(name).get("desired", False)
        if not desired:
            await log_hub.push(name, "watchdog", "WARN",
                               f"Exited ({returncode}), not desired — not restarting")
            return

        # Crash-restart budget: count ONLY crashes (this path), never manual
        # starts, so operator restarts don't exhaust the recovery budget.
        window = info["restart_window"]
        max_restarts = info["max_restarts"]
        info["restart_times"] = info.get("restart_times", []) + [time.time()]
        info["restarts"] = info.get("restarts", 0) + 1
        recent = [t for t in info["restart_times"] if time.time() - t < window]
        if len(recent) > max_restarts:
            await log_hub.push(name, "watchdog", "ERROR",
                               f"Too many restarts ({len(recent)} in {window}s) — giving up")
            self._state.set_desired(name, False)
            return

        await log_hub.push(name, "watchdog", "WARN", f"Exited ({returncode}), restarting in 1s...")
        await asyncio.sleep(1)
        try:
            await self.start(name)
        except Exception as e:
            await log_hub.push(name, "watchdog", "ERROR", f"Restart failed: {e}")

    async def reconcile(self) -> None:
        """Bring desired components up to match persisted desired-state. Called
        on startup so the manager recovers after a process or OS restart."""
        self.ensure_state()
        for name, comp in self._state.all_components().items():
            if not comp.get("desired", False):
                continue
            info = self._procs.get(name)
            running = info and info.get("proc") and info["proc"].returncode is None
            if running:
                continue
            extra = comp.get("extra") or None
            try:
                await self.start(name, extra)
            except Exception as e:
                await log_hub.push(name, "reconcile", "ERROR", f"reconcile start failed: {e}")

    def get_status(self) -> List[Dict[str, Any]]:
        components = []
        for name in COMPONENT_CONFIG.keys():
            info = self._procs.get(name, {})
            proc = info.get("proc")
            running = proc is not None and proc.returncode is None
            desired = self._state.get_component(name).get("desired", False)
            autostart = self._state.get_component(name).get("autostart", False)

            if running:
                level = 0
                level_name = "OK"
                message = f"RUNNING (pid {proc.pid})"
            elif desired:
                level = 2
                level_name = "ERROR"
                message = "STOPPED (expected running)"
            else:
                level = 0
                level_name = "OK"
                message = "NOT RUNNING (idle)"

            components.append({
                "name": f"terraslam/{name}",
                "level": level,
                "level_name": level_name,
                "message": message,
                "desired": desired,
                "autostart": autostart,
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
            await self.stop(name, force=True, clear_desired=False)


manager = ProcessManager()

# Пути, которые передаются отдельным запросом до старта компонента
publisher_folder_path: Optional[str] = None
