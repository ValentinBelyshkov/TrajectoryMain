"""Fast, hardware-free regression tests for the critical TerraSLAM contracts.

These tests intentionally do not start ROS, ffmpeg, MAVLink, or a real process.
They verify the validation and command-building boundaries where bad input or a
crashed component otherwise becomes a confusing UI failure.
"""

import asyncio
import importlib.util
import sys
import time
from pathlib import Path
from unittest.mock import AsyncMock, patch

import pytest


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parent
sys.path.insert(0, str(WORKSPACE))


def _load_module(name: str):
    try:
        return __import__(name, fromlist=["*"])
    except ModuleNotFoundError as exc:
        pytest.skip(f"runtime dependency is unavailable: {exc.name}")


def _load_file_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        pytest.fail(f"cannot load module from {path}")
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except ModuleNotFoundError as exc:
        pytest.skip(f"runtime dependency is unavailable: {exc.name}")
    return module


def test_image_frame_sorting_handles_expected_and_unexpected_names():
    image_publish = _load_file_module("image_publish", WORKSPACE / "Database/image_publish.py")

    names = ["frame_0010.jpg", "0002.jpg", "not-a-frame.jpg", "frame_0003.png"]
    assert sorted(names, key=image_publish._frame_index) == [
        "not-a-frame.jpg",
        "0002.jpg",
        "frame_0003.png",
        "frame_0010.jpg",
    ]


def test_image_publisher_command_requires_a_directory():
    process_manager = _load_module("TerraSLAM_relay.system_manager_pkg.process_manager")
    manager = process_manager.ProcessManager()

    with pytest.raises(ValueError, match="requires path"):
        manager._build_cmd("publisher_folder")


def test_image_publisher_command_quotes_paths_and_supports_once_mode():
    process_manager = _load_module("TerraSLAM_relay.system_manager_pkg.process_manager")
    manager = process_manager.ProcessManager()

    command = manager._build_cmd(
        "publisher_folder",
        {"path": "/tmp/frames with spaces", "procframe_dir": "/tmp/proc", "once": True},
    )

    assert command[0:2] == ["bash", "-lc"]
    assert "image_publish.py '/tmp/frames with spaces'" in command[2]
    assert "--procframe-dir /tmp/proc" in command[2]
    assert command[2].endswith("--once")


def test_component_status_marks_desired_stopped_process_as_error(tmp_path):
    process_manager = _load_module("TerraSLAM_relay.system_manager_pkg.process_manager")
    manager = process_manager.ProcessManager()
    manager.ensure_state(str(tmp_path))
    manager._state.set_desired("gps_client", True)
    manager._procs["gps_client"] = {
        "proc": None,
        "restarts": 1,
    }

    status = {item["values"]["program"]: item for item in manager.get_status()}

    assert status["gps_client"]["level_name"] == "ERROR"
    assert status["gps_client"]["message"] == "STOPPED (expected running)"


def test_watchdog_restarts_desired_component_after_crash(tmp_path):
    process_manager = _load_module("TerraSLAM_relay.system_manager_pkg.process_manager")

    class CrashedProcess:
        pid = 4242

        async def wait(self):
            return 139

    manager = process_manager.ProcessManager()
    manager.ensure_state(str(tmp_path))
    manager._state.set_desired("gps_client", True)
    manager._procs["gps_client"] = {
        "proc": CrashedProcess(),
        "restarts": 0,
        "restart_times": [],
        "restart_window": 60,
        "max_restarts": 3,
    }

    with patch.object(manager, "start", new_callable=AsyncMock) as start_mock:
        with patch("asyncio.sleep", new_callable=AsyncMock):
            asyncio.run(manager._watchdog("gps_client"))

    start_mock.assert_awaited_once_with("gps_client")
    assert manager._procs["gps_client"]["restarts"] == 1


def test_watchdog_stops_recovery_after_restart_budget_is_exhausted(tmp_path):
    process_manager = _load_module("TerraSLAM_relay.system_manager_pkg.process_manager")

    class CrashedProcess:
        pid = 4343

        async def wait(self):
            return 1

    manager = process_manager.ProcessManager()
    manager.ensure_state(str(tmp_path))
    manager._state.set_desired("gps_client", True)
    manager._procs["gps_client"] = {
        "proc": CrashedProcess(),
        "restarts": 3,
        "restart_times": [time.time(), time.time(), time.time()],
        "restart_window": 60,
        "max_restarts": 3,
    }

    with patch.object(manager, "start", new_callable=AsyncMock) as start_mock:
        asyncio.run(manager._watchdog("gps_client"))

    start_mock.assert_not_awaited()
    assert manager._state.get_component("gps_client")["desired"] is False


def test_gps_calibration_rejects_fewer_than_three_points(tmp_path):
    gps = _load_module("TerraSLAM_relay.system_manager_pkg.routes_gps")
    calib = tmp_path / "calib.gpc"
    calib.write_text("+proj=longlat\n0 0 0 37.0 55.0 100\n1 0 0 37.1 55.1 100\n")

    with pytest.raises(ValueError, match="Need"):
        gps._apply_calib(str(calib), 0.5, 0.5, 0.0)


def test_gps_calibration_returns_expected_coordinates(tmp_path):
    gps = _load_module("TerraSLAM_relay.system_manager_pkg.routes_gps")
    calib = tmp_path / "calib.gpc"
    # lat = 55 + x, lon = 37 + 2*y, constant altitude
    calib.write_text(
        "+proj=longlat\n"
        "0 0 0 37 55 120\n"
        "1 0 0 37 56 120\n"
        "0 1 0 39 55 120\n"
        "1 1 0 39 56 120\n"
    )

    lat, lon, alt = gps._apply_calib(str(calib), 0.25, 0.5, 0.0)

    assert lat == pytest.approx(55.25)
    assert lon == pytest.approx(38.0)
    assert alt == pytest.approx(120.0)