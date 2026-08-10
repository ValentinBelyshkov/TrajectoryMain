"""
Persistent desired-state store for the TerraSLAM System Manager.

Holds the "desired state" of components (which should be running, with what
extra args), the failsafe (fallback) configuration, the active publisher mode,
and any in-flight SLAM session. Persisted atomically to a JSON file under the
projects directory so the manager can reconcile and recover after a process or
OS restart.
"""
import json
import os
import tempfile
from typing import Any, Dict, Optional

from .config import COMPONENT_CONFIG

STATE_FILENAME = "manager_state.json"
STATE_VERSION = 1

# Components that should come up automatically on a (re)start. rosbridge is the
# manager's own transport and slam_mode_manager drives the SLAM lifecycle, so both
# are desired+autostart by default. Everything else is operator-driven.
AUTOSTART_DEFAULTS = {"rosbridge", "slam_mode_manager", "gps_client"}


def atomic_write(path: str, text: str) -> None:
    """Write text to path atomically (temp file + os.replace)."""
    d = os.path.dirname(path) or "."
    fd, tmp = tempfile.mkstemp(dir=d, prefix=".tmp_state_")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            try:
                os.unlink(tmp)
            except OSError:
                pass


class StateStore:
    def __init__(self, path: Optional[str] = None):
        self._path = path
        self._data: Dict[str, Any] = {}
        self._loaded = False

    # --- lifecycle ---
    def init(self, projects_path) -> None:
        self._path = os.path.join(str(projects_path), STATE_FILENAME)
        self._load_or_seed()

    def _default_components(self) -> Dict[str, Any]:
        comps = {}
        for name in COMPONENT_CONFIG:
            autostart = name in AUTOSTART_DEFAULTS
            comps[name] = {
                "desired": autostart,
                "autostart": autostart,
                "extra": {},
            }
        return comps

    def _fresh(self) -> Dict[str, Any]:
        return {
            "version": STATE_VERSION,
            "components": self._default_components(),
            "fallback": {
                "method": "RTL",
                "connection": "udp:127.0.0.1:14550",
            },
            "publisher_mode": "folder",
            "active_session": None,
        }

    def _load_or_seed(self) -> None:
        if self._loaded:
            return
        if self._path and os.path.isfile(self._path):
            try:
                with open(self._path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if isinstance(data, dict) and data.get("version") == STATE_VERSION:
                    self._data = data
                else:
                    self._data = self._fresh()
                    self._merge(data)
            except Exception:
                self._data = self._fresh()
        else:
            self._data = self._fresh()
        # ensure every known component has an entry
        for name, c in self._default_components().items():
            self._data.setdefault("components", {}).setdefault(name, c)
        self._loaded = True
        self._save()

    def _merge(self, old: Dict[str, Any]) -> None:
        if isinstance(old.get("fallback"), dict):
            self._data["fallback"].update(old["fallback"])
        if old.get("publisher_mode"):
            self._data["publisher_mode"] = old["publisher_mode"]
        if isinstance(old.get("components"), dict):
            for name, c in old["components"].items():
                if name in self._data["components"]:
                    self._data["components"][name].update(c)

    def _save(self) -> None:
        if not self._path:
            return
        atomic_write(self._path, json.dumps(self._data, indent=2, ensure_ascii=False))

    # --- components ---
    def get_component(self, name: str) -> Dict[str, Any]:
        return self._data.setdefault("components", {}).get(
            name, {"desired": False, "autostart": False, "extra": {}}
        )

    def set_desired(self, name: str, desired: bool, extra: Optional[Dict[str, Any]] = None) -> None:
        comp = self._data.setdefault("components", {}).setdefault(
            name, {"desired": False, "autostart": False, "extra": {}}
        )
        comp["desired"] = desired
        if extra is not None:
            comp["extra"] = extra
        self._save()

    def set_autostart(self, name: str, autostart: bool) -> None:
        comp = self._data.setdefault("components", {}).setdefault(
            name, {"desired": False, "autostart": False, "extra": {}}
        )
        comp["autostart"] = autostart
        if autostart:
            comp["desired"] = True
        self._save()

    def set_extra(self, name: str, extra: Dict[str, Any]) -> None:
        comp = self._data.setdefault("components", {}).setdefault(
            name, {"desired": False, "autostart": False, "extra": {}}
        )
        comp["extra"] = extra
        self._save()

    def reset_components_to_defaults(self) -> None:
        """Reset all component desired/autostart state to the defaults (only
        AUTOSTART_DEFAULTS are desired+autostart, every other component is off).
        Called on startup so the launch state is deterministic regardless of any
        previously persisted desired-state."""
        self._data["components"] = self._default_components()
        self._save()

    # --- components ---
    def all_components(self) -> Dict[str, Any]:
        return self._data.setdefault("components", {})
    def get_fallback(self) -> Dict[str, str]:
        return self._data.setdefault(
            "fallback", {"method": "RTL", "connection": "udp:127.0.0.1:14550"}
        )

    def set_fallback(self, method: Optional[str] = None, connection: Optional[str] = None) -> None:
        fb = self._data.setdefault("fallback", {})
        if method is not None:
            fb["method"] = method
        if connection is not None:
            fb["connection"] = connection
        self._save()

    # --- publisher mode ---
    def get_publisher_mode(self) -> str:
        return self._data.get("publisher_mode", "folder")

    def set_publisher_mode(self, mode: str) -> None:
        self._data["publisher_mode"] = mode
        self._save()

    # --- session ---
    def get_session(self) -> Optional[Dict[str, Any]]:
        return self._data.get("active_session")

    def set_session(self, rec: Optional[Dict[str, Any]]) -> None:
        self._data["active_session"] = rec
        self._save()

    def clear_session(self) -> None:
        self._data["active_session"] = None
        self._save()


state_store = StateStore()
