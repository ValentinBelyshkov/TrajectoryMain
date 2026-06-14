#!/usr/bin/env python3
"""
TerraSLAM Host Client
Синхронный HTTP-клиент для управления TerraSLAM с хоста.
Никакого ROS2 не требуется.
"""
import os
import sys
import time
import argparse
from typing import Optional

try:
    import requests
except ImportError:
    print("pip install requests")
    sys.exit(1)

URL = os.getenv("TERRASLAM_GATEWAY_URL", "http://localhost:8000").rstrip("/")

C = {
    "ok": "\033[92m", "warn": "\033[93m", "err": "\033[91m",
    "info": "\033[94m", "bold": "\033[1m", "reset": "\033[0m",
}

LEVEL = {0: ("OK", "ok"), 1: ("WARN", "warn"), 2: ("ERROR", "err"), 3: ("STALE", "warn")}


def _get(path: str, params=None):
    r = requests.get(f"{URL}{path}", params=params, timeout=10)
    r.raise_for_status()
    return r.json()


def _post(path: str, json=None):
    r = requests.post(f"{URL}{path}", json=json, timeout=15)
    r.raise_for_status()
    return r.json()


def cmd_status(args):
    data = _get("/api/v1/status")
    print(f"\n{C['bold']}TerraSLAM Status{C['reset']}")
    print("-" * 70)
    print(f"{'Component':<<30} {'State':<<10} {'Message'}")
    print("-" * 70)

    for c in data.get("components", []):
        name = c["name"].replace("terraslam/", "")
        lvl_name, color_key = LEVEL.get(c["level"], ("?", "reset"))
        color = C.get(color_key, "")
        msg = c.get("message", "")
        print(f"{color}{name:<30} {lvl_name:<10} {msg}{C['reset']}")

        vals = c.get("values", {})
        if vals:
            for k, v in vals.items():
                print(f"    {C['info']}{k}{C['reset']}: {v}")
    print("-" * 70)
    age = time.time() - data.get("timestamp", 0)
    print(f"Status age: {age:.1f}s\n")


def cmd_logs(args):
    params = {"limit": args.limit}
    if args.component != "all":
        params["component"] = args.component
    data = _get("/api/v1/logs", params=params)

    if not args.follow:
        print(f"\n{C['bold']}Last {data['count']} log lines{C['reset']}\n")
        for l in data.get("logs", []):
            _print_log(l)
        return

    # follow mode — polling
    print(f"\n{C['bold']}Following logs ({args.component})... Ctrl+C to stop{C['reset']}\n")
    seen = set()
    for l in data.get("logs", []):
        _print_log(l)
        seen.add(l.get("timestamp", 0))

    try:
        while True:
            time.sleep(1.0)
            new_data = _get("/api/v1/logs", params={**params, "since": max(seen) if seen else None})
            for l in new_data.get("logs", []):
                ts = l.get("timestamp", 0)
                if ts not in seen:
                    _print_log(l)
                    seen.add(ts)
                    if len(seen) > 5000:
                        seen = set(list(seen)[-2000:])
    except KeyboardInterrupt:
        print(f"\n{C['info']}Stopped.{C['reset']}")


def _print_log(l: dict):
    level = l.get("level", "INFO")
    comp = l.get("component", "unknown")
    source = l.get("source", "")
    msg = l.get("message", "")
    ts = time.strftime("%H:%M:%S", time.localtime(l.get("timestamp", 0)))
    color = C.get(level.lower(), "")
    if level == "ERROR":
        color = C["err"]
    elif level == "WARN":
        color = C["warn"]
    print(f"{color}[{ts}][{level:5}][{comp:15}][{source:7}] {msg}{C['reset']}")


def cmd_action(args):
    action = args.action
    comp = args.component
    payload = {}

    if action == "mode":
        r = _post("/api/v1/publisher/mode", json={"mode": comp})
    elif action == "path":
        r = _post("/api/v1/publisher/path", json={"path": comp})
    else:
        r = _post(f"/api/v1/components/{comp}/{action}", json=payload)

    ok = r.get("success", False)
    color = C["ok"] if ok else C["err"]
    print(f"{color}[{r.get('status', '?').upper()}] {r.get('message', '')}{C['reset']}")


def main():
    parser = argparse.ArgumentParser(description="TerraSLAM Host HTTP Client")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("status", help="Show component status")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("logs", help="Show recent logs")
    p.add_argument("component", nargs="?", default="all", help="Component name or 'all'")
    p.add_argument("-f", "--follow", action="store_true", help="Poll for new logs")
    p.add_argument("-n", "--limit", type=int, default=50)
    p.set_defaults(func=cmd_logs)

    for action in ("start", "stop", "restart", "kill"):
        p = sub.add_parser(action, help=f"{action.capitalize()} component")
        p.add_argument("component", help="Component name")
        p.set_defaults(func=cmd_action, action=action)

    p = sub.add_parser("mode", help="Switch publisher mode")
    p.add_argument("component", choices=["folder", "realsense"], help="Mode")
    p.set_defaults(func=cmd_action, action="mode")

    p = sub.add_parser("path", help="Set publisher folder path")
    p.add_argument("component", help="Absolute path inside container")
    p.set_defaults(func=cmd_action, action="path")

    args = parser.parse_args()
    try:
        args.func(args)
    except requests.exceptions.ConnectionError:
        print(f"{C['err']}Cannot connect to {URL}{C['reset']}")
        print("Set TERRASLAM_GATEWAY_URL or check that gateway is running.")
        sys.exit(1)
    except requests.exceptions.HTTPError as e:
        print(f"{C['err']}HTTP {e.response.status_code}: {e.response.text}{C['reset']}")
        sys.exit(1)


if __name__ == "__main__":
    main()
