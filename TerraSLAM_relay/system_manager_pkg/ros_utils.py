"""Shared helper for running one-off `ros2` CLI commands."""
import os
import subprocess
from typing import List, Tuple


def ros2_run(ros2_args: List[str], timeout: int = 5) -> Tuple[str, str, int]:
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
