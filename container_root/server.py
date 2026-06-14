#!/usr/bin/env python3
"""
TerraSLAM Gateway — Flask HTTP server (port 5000).
Serves the web dashboard and proxies to system_manager (port 9000).
Also exposes legacy task management endpoints.
"""
from flask import Flask, jsonify, request, render_template
import subprocess
import threading
import time
import os
import uuid
import signal
import sys
import json
from datetime import datetime
from functools import partial

try:
    import urllib.request as ureq
    import urllib.error as uerr
except ImportError:
    pass

app = Flask(__name__, template_folder='templates', static_folder='static')

# ─── Globals ──────────────────────────────────────────────────────────────────
tasks = {}
last_tasks_by_type = {}
server_shutting_down = False

SM_BASE = "http://localhost:9000"   # system_manager FastAPI


# ─── Helpers ──────────────────────────────────────────────────────────────────
def sm_get(path, params=None):
    """GET request to system_manager."""
    url = SM_BASE + path
    if params:
        qs = "&".join(f"{k}={v}" for k, v in params.items())
        url += "?" + qs
    req = ureq.Request(url)
    try:
        with ureq.urlopen(req, timeout=5) as r:
            return json.loads(r.read()), r.status
    except uerr.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        return {"error": body}, e.code
    except Exception as e:
        return {"error": str(e), "sm_unavailable": True}, 503


def sm_post(path, payload=None):
    """POST request to system_manager."""
    url = SM_BASE + path
    data = json.dumps(payload or {}).encode("utf-8")
    req = ureq.Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        with ureq.urlopen(req, timeout=10) as r:
            return json.loads(r.read()), r.status
    except uerr.HTTPError as e:
        body = e.read().decode("utf-8", errors="replace")
        return {"error": body}, e.code
    except Exception as e:
        return {"error": str(e), "sm_unavailable": True}, 503


def ros2_cmd_once(ros2_args, timeout=3):
    """Run a ros2 command and return (stdout, stderr, returncode)."""
    cmd = ["ros2"] + ros2_args
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout,
            env={**os.environ, "ROS_DOMAIN_ID": "0"}
        )
        return proc.stdout.strip(), proc.stderr.strip(), proc.returncode
    except FileNotFoundError:
        return "", "ros2 not found — run inside the Docker container", 127
    except subprocess.TimeoutExpired:
        return "", f"Timeout after {timeout}s", 1
    except Exception as e:
        return "", str(e), 1


# ─── Dashboard ────────────────────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("index.html")


# ─── Health ───────────────────────────────────────────────────────────────────
@app.route("/health")
def health():
    sm_ok = sm_get("/health")[1] == 200
    return jsonify(status="ok", gateway="running", system_manager="ok" if sm_ok else "unavailable")


# ═══════════════════════════════════════════════════════════════════════════════
#  SYSTEM MANAGER PROXY  (/api/sm/…)
# ═══════════════════════════════════════════════════════════════════════════════

@app.route("/api/sm/status")
def sm_status():
    data, code = sm_get("/api/v1/status")
    return jsonify(data), code


@app.route("/api/sm/components/<component>/<action>", methods=["POST"])
def sm_component_action(component, action):
    payload = request.get_json(silent=True) or {}
    data, code = sm_post(f"/api/v1/components/{component}/{action}", payload)
    return jsonify(data), code


@app.route("/api/sm/slam/status")
def sm_slam_status():
    data, code = sm_get("/api/v1/slam/status")
    return jsonify(data), code


@app.route("/api/sm/slam/mode", methods=["POST"])
def sm_slam_mode():
    payload = request.get_json(silent=True) or {}
    data, code = sm_post("/api/v1/slam/mode", payload)
    return jsonify(data), code


@app.route("/api/sm/slam/command", methods=["POST"])
def sm_slam_command():
    payload = request.get_json(silent=True) or {}
    data, code = sm_post("/api/v1/slam/command", payload)
    return jsonify(data), code


@app.route("/api/sm/publisher/mode", methods=["POST"])
def sm_publisher_mode():
    payload = request.get_json(silent=True) or {}
    data, code = sm_post("/api/v1/publisher/mode", payload)
    return jsonify(data), code


@app.route("/api/sm/publisher/path", methods=["POST"])
def sm_publisher_path():
    payload = request.get_json(silent=True) or {}
    data, code = sm_post("/api/v1/publisher/path", payload)
    return jsonify(data), code


@app.route("/api/sm/logs")
def sm_logs():
    params = {}
    if request.args.get("component"):
        params["component"] = request.args["component"]
    if request.args.get("limit"):
        params["limit"] = request.args["limit"]
    if request.args.get("since"):
        params["since"] = request.args["since"]
    data, code = sm_get("/api/v1/logs", params)
    return jsonify(data), code


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS2 DIRECT COMMANDS  (/api/ros2/…)
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


@app.route("/api/ros2/topic/<name>")
def ros2_topic_echo(name):
    """Echo one message from a known ROS2 topic."""
    topic = ROS2_TOPICS.get(name)
    if not topic:
        return jsonify(error=f"Unknown topic '{name}'. Known: {list(ROS2_TOPICS.keys())}"), 400
    stdout, stderr, rc = ros2_cmd_once(
        ["topic", "echo", "--once", topic], timeout=5
    )
    return jsonify(topic=topic, output=stdout, error=stderr, returncode=rc)


@app.route("/api/ros2/topic_list")
def ros2_topic_list():
    """List all active ROS2 topics."""
    stdout, stderr, rc = ros2_cmd_once(["topic", "list"], timeout=5)
    topics = [t.strip() for t in stdout.splitlines() if t.strip()] if rc == 0 else []
    return jsonify(topics=topics, error=stderr, returncode=rc)


@app.route("/api/ros2/node_list")
def ros2_node_list():
    """List all active ROS2 nodes."""
    stdout, stderr, rc = ros2_cmd_once(["node", "list"], timeout=5)
    nodes = [n.strip() for n in stdout.splitlines() if n.strip()] if rc == 0 else []
    return jsonify(nodes=nodes, error=stderr, returncode=rc)


@app.route("/api/ros2/mavros/arm", methods=["POST"])
def mavros_arm():
    """Send ARM command via mavros service."""
    body = request.get_json(silent=True) or {}
    arm = body.get("arm", True)
    val = "true" if arm else "false"
    stdout, stderr, rc = ros2_cmd_once(
        ["service", "call", "/mavros/cmd/arming",
         "mavros_msgs/srv/CommandBool", f"{{value: {val}}}"], timeout=8
    )
    return jsonify(output=stdout, error=stderr, returncode=rc)


@app.route("/api/ros2/mavros/set_mode", methods=["POST"])
def mavros_set_mode():
    """Set flight mode via mavros service."""
    body = request.get_json(silent=True) or {}
    mode = body.get("mode", "GUIDED")
    stdout, stderr, rc = ros2_cmd_once(
        ["service", "call", "/mavros/set_mode",
         "mavros_msgs/srv/SetMode", f'{{custom_mode: "{mode}"}}'], timeout=8
    )
    return jsonify(output=stdout, error=stderr, returncode=rc)


@app.route("/api/ros2/slam/start", methods=["POST"])
def ros2_slam_start():
    """Start ORB-SLAM3 via ROS2 launch (legacy path, no system_manager)."""
    task_id = str(uuid.uuid4())
    cmd = "ros2 launch orb_slam3_ros2_wrapper unirobot_mono.launch.py"
    cwd = "/root/colcon_ws"
    tasks[task_id] = {
        "type": "slam",
        "cmd": cmd,
        "cwd": cwd,
        "status": "starting",
        "stdout": "",
        "stderr": "",
    }
    last_tasks_by_type["slam"] = task_id
    thread = threading.Thread(target=_run_task, args=(task_id, cmd, cwd), daemon=True)
    thread.start()
    return jsonify(task_id=task_id, status="started", note="ROS2 direct launch")


# ═══════════════════════════════════════════════════════════════════════════════
#  LEGACY TASK MANAGER  (/start  /status  /list  /stop)
# ═══════════════════════════════════════════════════════════════════════════════

def _run_task(task_id, cmd, cwd):
    try:
        if server_shutting_down:
            tasks[task_id]["status"] = "cancelled"
            return
        proc = subprocess.Popen(
            cmd, shell=True, cwd=cwd,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1, universal_newlines=True,
            start_new_session=True,
        )
        tasks[task_id]["proc"] = proc
        tasks[task_id]["start_time"] = datetime.utcnow().isoformat()
        task_type = tasks[task_id]["type"]
        last_tasks_by_type[task_type] = task_id

        stdout_lines, stderr_lines = [], []

        def read_stdout():
            for line in iter(proc.stdout.readline, ""):
                if server_shutting_down:
                    break
                stdout_lines.append(line)
                tasks[task_id]["stdout"] = "".join(stdout_lines[-100:])

        def read_stderr():
            for line in iter(proc.stderr.readline, ""):
                if server_shutting_down:
                    break
                stderr_lines.append(line)
                tasks[task_id]["stderr"] = "".join(stderr_lines[-100:])

        threading.Thread(target=read_stdout, daemon=True).start()
        threading.Thread(target=read_stderr, daemon=True).start()

        while proc.poll() is None:
            if server_shutting_down:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=2)
                except Exception:
                    pass
                break
            time.sleep(0.1)

        if not server_shutting_down:
            tasks[task_id]["returncode"] = proc.returncode
            tasks[task_id]["status"] = "finished" if proc.returncode == 0 else "failed"
            tasks[task_id]["end_time"] = datetime.utcnow().isoformat()
    except Exception as e:
        if not server_shutting_down:
            tasks[task_id]["status"] = "error"
            tasks[task_id]["error"] = str(e)


@app.route("/start", methods=["POST"])
def start_task():
    data = request.json or {}
    task_type = data.get("type")
    TASK_CMDS = {
        "slam":      ("ros2 launch orb_slam3_ros2_wrapper unirobot_mono.launch.py", "/root/colcon_ws"),
        "image_pub": (f"python3 image_publish.py {data.get('video', 'Mill-video')}/", "/root/Database"),
        "relay":     ("python3 relay.py", "/root/TerraSLAM_relay"),
        "callback":  ("python counter_callback.py", "."),
    }
    if task_type not in TASK_CMDS:
        return jsonify(error="Unknown task type"), 400
    cmd, cwd = TASK_CMDS[task_type]
    task_id = str(uuid.uuid4())
    tasks[task_id] = {"type": task_type, "cmd": cmd, "cwd": cwd, "status": "starting", "stdout": "", "stderr": ""}
    last_tasks_by_type[task_type] = task_id
    threading.Thread(target=_run_task, args=(task_id, cmd, cwd), daemon=True).start()
    return jsonify(task_id=task_id, status="started")


KNOWN_TYPES = {"slam", "image_pub", "relay", "callback"}


@app.route("/status/<task_id_or_type>")
def get_status(task_id_or_type):
    if task_id_or_type in KNOWN_TYPES:
        last = last_tasks_by_type.get(task_id_or_type)
        if not last or last not in tasks:
            return jsonify(error="No task of this type found"), 404
        task = tasks[last]
    else:
        task = tasks.get(task_id_or_type)
        if not task:
            return jsonify(error="Task not found"), 404
    proc = task.get("proc")
    if proc and proc.poll() is None:
        task["status"] = "running"
    return jsonify({k: v for k, v in task.items() if k != "proc"})


@app.route("/list")
def list_tasks():
    return jsonify({
        tid: {k: v for k, v in t.items() if k not in ("stdout", "stderr", "proc")}
        for tid, t in tasks.items()
    })


@app.route("/stop/<task_id_or_type>", methods=["POST"])
def stop_task(task_id_or_type):
    if task_id_or_type in KNOWN_TYPES:
        last = last_tasks_by_type.get(task_id_or_type)
        if not last or last not in tasks:
            return jsonify(error="No task of this type found"), 404
        task = tasks[last]
        task_ref = last
    else:
        task = tasks.get(task_id_or_type)
        if not task:
            return jsonify(error="Task not found"), 404
        task_ref = task_id_or_type

    proc = task.get("proc")
    if not proc:
        return jsonify(error="Task has no process"), 400
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        task["status"] = "stopped"
        task["end_time"] = datetime.utcnow().isoformat()
        return jsonify(message=f"Task {task_ref} stopped")
    except Exception as e:
        return jsonify(error=str(e)), 500


# ─── Shutdown ─────────────────────────────────────────────────────────────────
def shutdown_handler(signum, frame):
    global server_shutting_down
    server_shutting_down = True
    for task_id, task_data in tasks.items():
        if task_data.get("status") in ("running", "starting"):
            proc = task_data.get("proc")
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=5)
                except Exception:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except Exception:
                        pass
    sys.exit(0)


signal.signal(signal.SIGTERM, shutdown_handler)
signal.signal(signal.SIGINT, shutdown_handler)
if hasattr(signal, "SIGQUIT"):
    signal.signal(signal.SIGQUIT, shutdown_handler)


if __name__ == "__main__":
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.makedirs(os.path.join(base, "Database"), exist_ok=True)
    os.makedirs(os.path.join(base, "TerraSLAM_relay"), exist_ok=True)
    app.run(host="0.0.0.0", port=5000, threaded=True)
