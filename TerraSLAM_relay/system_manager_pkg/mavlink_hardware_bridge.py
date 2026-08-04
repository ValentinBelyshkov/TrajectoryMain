"""
MAVLink Hardware Bridge — sends custom MAVLink commands over UART to
companion hardware and listens for responses.

Protocol (MAVLink v2, NAMED_VALUE_FLOAT msgid 28):
  - Commands are sent as NAMED_VALUE_FLOAT with name="ORB_CMD"
    value=1.0 (start), 2.0 (stop), 3.0 (restart)
  - Responses are received as NAMED_VALUE_FLOAT with name="ORB_RES"
    value=0.0 (success), 1.0 (error)
  - Additional status text is sent as STATUSTEXT (msgid 74)

The bridge runs a background reader thread so it can asynchronously
collect hardware responses, and is thread-safe for concurrent sends
from FastAPI request handlers.
"""
import os
import threading
import time
from typing import Any, Dict, Optional

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except ImportError:
    mavutil = None
    PYMAVLINK_AVAILABLE = False

CMD_START = 1.0
CMD_STOP = 2.0
CMD_RESTART = 3.0
CMD_VALUES = {"start": CMD_START, "stop": CMD_STOP, "restart": CMD_RESTART}

CMD_NAME = "ORB_CMD"
RESP_NAME = "ORB_RES"


class MavlinkHardwareBridge:
    """MAVLink UART bridge for communicating with custom hardware.

    Sends custom MAVLink commands over UART and listens for responses.
    Uses NAMED_VALUE_FLOAT for the command/response protocol.
    """

    def __init__(self, port: str, baudrate: int = 115200, sys_id: int = 1):
        self.port = port
        self.baudrate = baudrate
        self.sys_id = sys_id
        self._master = None
        self._lock = threading.Lock()
        self._reader_thread: Optional[threading.Thread] = None
        self._running = False
        self._last_response: Optional[Dict[str, Any]] = None
        self._last_response_time: float = 0.0
        self._last_error: Optional[str] = None
        self._connected = False
        self._log_callback = None
        self._response_event = threading.Event()

    def set_log_callback(self, callback):
        """Set a synchronous callback for logging (e.g. to LogHub)."""
        self._log_callback = callback

    def _log(self, level: str, message: str):
        if self._log_callback:
            try:
                self._log_callback("mavlink_bridge", "uart", level, message)
            except Exception:
                pass

    def connect(self) -> bool:
        """Open the UART port and start the background reader thread."""
        if not PYMAVLINK_AVAILABLE:
            self._last_error = "pymavlink is not installed"
            self._log("ERROR", self._last_error)
            return False

        try:
            if self._master is not None:
                try:
                    self._master.close()
                except Exception:
                    pass

            self._master = mavutil.mavlink_connection(
                self.port, baud=self.baudrate, source_system=self.sys_id
            )
            self._connected = True
            self._last_error = None
            self._log("INFO", f"Connected to UART {self.port} at {self.baudrate} baud")

            self._running = True
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._reader_thread.start()
            return True
        except Exception as e:
            self._last_error = f"Failed to connect to UART {self.port}: {e}"
            self._log("ERROR", self._last_error)
            self._connected = False
            return False

    def disconnect(self):
        """Close the UART port and stop the reader thread."""
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=2.0)
            self._reader_thread = None
        if self._master:
            try:
                self._master.close()
            except Exception:
                pass
            self._master = None
        self._connected = False
        self._log("INFO", "Disconnected from UART")

    def _read_loop(self):
        """Background thread — continuously read and dispatch MAVLink messages."""
        while self._running and self._master:
            try:
                msg = self._master.recv_match(blocking=True, timeout=1.0)
                if msg:
                    self._handle_message(msg)
            except Exception as e:
                self._log("ERROR", f"MAVLink read error: {e}")
                time.sleep(0.1)

    def _handle_message(self, msg):
        """Dispatch an incoming MAVLink message to the appropriate handler."""
        msg_type = msg.get_type()
        msg_name = msg_type.name if hasattr(msg_type, "name") else str(msg_type)

        if msg_name == "NAMED_VALUE_FLOAT":
            name = msg.name.decode("utf-8", errors="replace").rstrip("\x00").strip()
            value = msg.value

            if name == RESP_NAME:
                self._last_response = {
                    "name": name,
                    "value": value,
                    "success": value == 0.0,
                    "timestamp": time.time(),
                }
                self._last_response_time = time.time()
                self._log("INFO", f"Hardware response: {name}={value}")
                self._response_event.set()

        elif msg_name == "STATUSTEXT":
            text = msg.text.decode("utf-8", errors="replace").rstrip("\x00").strip()
            self._log("INFO", f"Hardware status: {text}")

    def send_slam_control(self, action: str, timeout: float = 5.0) -> Dict[str, Any]:
        """Send a SLAM control command to the hardware over UART.

        Args:
            action: "start", "stop", or "restart"
            timeout: seconds to wait for a hardware response (0 = don't wait)

        Returns:
            Dict with success status, the command sent, and a message.
            If a response arrives within *timeout*, it is included as
            ``result["response"]``.
        """
        if not PYMAVLINK_AVAILABLE:
            return {"success": False, "error": "pymavlink is not installed"}

        with self._lock:
            if self._master is None or not self._connected:
                if not self.connect():
                    return {"success": False, "error": self._last_error or "Not connected to UART"}

            cmd_value = CMD_VALUES.get(action)
            if cmd_value is None:
                return {
                    "success": False,
                    "error": f"Unknown action: {action}. Use start/stop/restart.",
                }

            self._response_event.clear()
            self._last_response = None
            self._last_response_time = 0.0

            try:
                self._master.mav.named_value_float_send(
                    int(time.time() * 1000) & 0xFFFFFFFF,
                    0,
                    cmd_value,
                    CMD_NAME.encode("utf-8"),
                )
                self._log("INFO", f"Sent SLAM command: {action} (value={cmd_value})")

                result = {
                    "success": True,
                    "action": action,
                    "command": CMD_NAME,
                    "value": cmd_value,
                    "message": f"MAVLink command '{action}' sent to hardware via UART {self.port}",
                }

                if timeout > 0:
                    self._response_event.wait(timeout=timeout)
                    if self._last_response is not None:
                        result["response"] = self._last_response
                        result["message"] += f" — response: {self._last_response}"
                    else:
                        result["response"] = None
                        result["message"] += " (no response received within timeout)"

                return result
            except Exception as e:
                self._log("ERROR", f"Failed to send MAVLink command: {e}")
                return {"success": False, "error": str(e)}

    def get_status(self) -> Dict[str, Any]:
        """Return the current bridge state and the last response from hardware."""
        age = None
        if self._last_response_time > 0:
            age = time.time() - self._last_response_time

        return {
            "connected": self._connected,
            "port": self.port,
            "baudrate": self.baudrate,
            "pymavlink_available": PYMAVLINK_AVAILABLE,
            "last_response": self._last_response,
            "last_response_age_seconds": round(age, 3) if age is not None else None,
            "last_error": self._last_error,
        }

    def reconnect(self) -> bool:
        """Force-close and re-open the UART connection."""
        self.disconnect()
        time.sleep(0.5)
        return self.connect()


mavlink_bridge: Optional[MavlinkHardwareBridge] = None


def get_mavlink_bridge() -> Optional[MavlinkHardwareBridge]:
    """Get the singleton MAVLink hardware bridge instance."""
    return mavlink_bridge


def init_mavlink_bridge(
    port: str,
    baudrate: int,
    sys_id: int,
    log_callback=None,
) -> Optional[MavlinkHardwareBridge]:
    """Create and connect the singleton MAVLink hardware bridge.

    Returns the bridge instance (or None on failure). The caller is
    responsible for storing the returned object if a handle is needed.
    """
    global mavlink_bridge
    mavlink_bridge = MavlinkHardwareBridge(port, baudrate, sys_id)
    if log_callback:
        mavlink_bridge.set_log_callback(log_callback)
    mavlink_bridge.connect()
    return mavlink_bridge
