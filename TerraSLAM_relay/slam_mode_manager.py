"""Manage ORB-SLAM3 run modes against the NEW ORB-SLAM3 ROS2 wrapper.

The new wrapper removed the old `map_control` service. Modes are now driven by:

  * /orb_slam3/save_map        (std_srvs/SetBool)  -> saveAtlas()  (save .osa)
  * /orb_slam3/reset_mapping   (std_srvs/SetBool)  -> resetLocalMapping()
  * /orb_slam3/slam_info       (slam_msgs/SlamInfo) -> status (tracking_frequency,
                                                        num_keyframes_in_current_map)
  * /orb_slam3/robot_pose_slam (PoseStamped)        -> live pose

Localization vs Mapping is selected via the ORB-SLAM3 settings YAML:
  * MAPPING           = no LoadAtlasFromFile  (build a fresh map)
  * LOCALIZATION_ONLY = LoadAtlasFromFile: "<map>.osa"  (relocalize on existing map)

Switching modes rewrites the YAML (via yaml_editor) and restarts the slam
component so the new setting takes effect.
"""
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import SetBool
from slam_msgs.msg import SlamInfo
import json
import os
import sys

# Allow importing the sibling package for yaml editing / process control.
sys.path.insert(0, "/opt/main/Trajectory/TerraSLAM_relay/system_manager_pkg")
from utils.yaml_editor import update_yaml  # type: ignore

PROJECTS_DIR = os.getenv("PROJECTS_DIR", "/opt/main/Trajectory/Database/projects")
# Path passed to ORB-SLAM3's System.SaveAtlasToFile. ORB-SLAM3 writes to
# "./<SaveAtlasToFile>.osa" relative to the SLAM process CWD (/opt/main/Trajectory),
# so this MUST be relative to that directory (no leading slash, no ".osa" suffix).
DEFAULT_MAP_OSA = "Database/projects/calibrations/map"
ORB_SETTINGS_YAML = "/opt/main/Trajectory/Database/real.yaml"

# Wrapper service/topic names (new wrapper).
SAVE_MAP_SRV = "/orb_slam3/save_map"
RESET_MAP_SRV = "/orb_slam3/reset_mapping"
SLAM_INFO_TOPIC = "/orb_slam3/slam_info"

MODE_FILE = "/tmp/terraslam_slam_mode"
CMD_FILE = "/tmp/terraslam_slam_cmd"
PATH_FILE = "/tmp/terraslam_slam_path"

# Status sentinel from config: pose == (-3,-3,-3) means tracking lost.
TRACKING_LOST_SENTINEL = -3.0


class SlamModeManager(Node):
    def __init__(self):
        super().__init__("slam_mode_manager")

        self.current_mode = "LOCALIZATION_ONLY"
        self.slam_state_name = "UNKNOWN"
        self.tracking_frequency = 0.0
        self.num_keyframes = 0
        self.initialized = False
        self._slam_save_up = False
        self._slam_reset_up = False

        # Publishers
        self.status_pub = self.create_publisher(String, "/slam/mode_status", 10)
        self.drone_cmd_pub = self.create_publisher(String, "/drone/mode_command", 10)

        # Service clients (new wrapper)
        self.save_map_cli = self.create_client(SetBool, SAVE_MAP_SRV)
        self.reset_map_cli = self.create_client(SetBool, RESET_MAP_SRV)

        # Status subscription (replaces old /orb_slam3/tracking_state Int8)
        self.create_subscription(SlamInfo, SLAM_INFO_TOPIC, self.slam_info_cb, 10)

        # Best-effort wait for the wrapper services.
        for svc, flag in ((self.save_map_cli, "_slam_save_up"), (self.reset_map_cli, "_slam_reset_up")):
            if svc.wait_for_service(timeout_sec=10.0):
                setattr(self, flag, True)
            else:
                self.get_logger().error(f"Service {svc.srv_name} not available at startup — will retry via timer.")

        self.create_timer(0.2, self.check_file_flags)
        self.publish_status()

    # ---- service wrappers ----
    def _call_setbool(self, client, flag_attr, name, data=True):
        if not client.service_is_ready():
            self.get_logger().warn(f"{name}: service not ready — skipping")
            return
        future = client.call_async(SetBool.Request(data=data))
        future.add_done_callback(lambda f: self._setbool_done(f, name))

    def _setbool_done(self, future, name):
        try:
            res = future.result()
            self.get_logger().info(f"{name}: success={res.success} message='{res.message}'")
        except Exception as e:
            self.get_logger().error(f"{name} exception: {e}")

    # ---- status from slam_info ----
    def slam_info_cb(self, msg: SlamInfo):
        self.tracking_frequency = float(msg.tracking_frequency)
        self.num_keyframes = int(msg.num_keyframes_in_current_map)
        # No explicit state in SlamInfo; derive a coarse status.
        if self.tracking_frequency > 0.1:
            self.slam_state_name = "OK"
        elif self.num_keyframes > 0:
            self.slam_state_name = "RECENTLY_LOST"
        else:
            self.slam_state_name = "NO_MAP"
        self.publish_status()

    # ---- file-flag control (written by the API layer) ----
    def check_file_flags(self):
        # Service availability tracking.
        self._slam_save_up = self.save_map_cli.service_is_ready()
        self._slam_reset_up = self.reset_map_cli.service_is_ready()

        # Mode switch (mapping / localization) via YAML + slam restart.
        if os.path.exists(MODE_FILE):
            try:
                with open(MODE_FILE, "r") as f:
                    mode = f.read().strip()
                if mode == "LOCALIZATION_ONLY" and self.current_mode != "LOCALIZATION_ONLY":
                    self.get_logger().info("→ LOCALIZATION_ONLY (load atlas, restart slam)")
                    self._set_mode_localization()
                elif mode == "SLAM_MAPPING" and self.current_mode != "SLAM_MAPPING":
                    self.get_logger().info("→ SLAM_MAPPING (no atlas, restart slam)")
                    self._set_mode_mapping()
                os.remove(MODE_FILE)
            except Exception as e:
                self.get_logger().error(f"Mode file error: {e}")

        # Direct commands (reset / save_map / load_map).
        if os.path.exists(CMD_FILE):
            try:
                with open(CMD_FILE, "r") as f:
                    cmd = f.read().strip()
                filepath = ""
                if os.path.exists(PATH_FILE):
                    with open(PATH_FILE, "r") as f:
                        filepath = f.read().strip()
                if cmd == "reset":
                    self._call_setbool(self.reset_map_cli, "_slam_reset_up", "reset_mapping")
                elif cmd == "save_map":
                    self._save_map(filepath or DEFAULT_MAP_OSA)
                elif cmd == "load_map":
                    self._set_mode_localization(filepath or DEFAULT_MAP_OSA)
                os.remove(CMD_FILE)
                if os.path.exists(PATH_FILE):
                    os.remove(PATH_FILE)
            except Exception as e:
                self.get_logger().error(f"Cmd file error: {e}")

    def _set_mode_localization(self, map_osa: str = DEFAULT_MAP_OSA):
        try:
            update_yaml(ORB_SETTINGS_YAML, None, load_filename=map_osa)
            self.get_logger().info(f"YAML: LoadAtlasFromFile -> {map_osa}")
        except Exception as e:
            self.get_logger().error(f"update_yaml (load) failed: {e}")
        self._restart_slam()
        self.current_mode = "LOCALIZATION_ONLY"

    def _set_mode_mapping(self):
        try:
            update_yaml(ORB_SETTINGS_YAML, None, load_filename=None)
            self.get_logger().info("YAML: removed LoadAtlasFromFile (mapping mode)")
        except Exception as e:
            self.get_logger().error(f"update_yaml (mapping) failed: {e}")
        self._restart_slam()
        self.current_mode = "SLAM_MAPPING"

    def _save_map(self, map_osa: str):
        # ORB-SLAM3 reads System.SaveAtlasToFile at startup, so the path is
        # already fixed in real.yaml (relative to the SLAM CWD). We must NOT
        # rewrite the YAML here (it wouldn't take effect until a restart) — just
        # trigger the save via the wrapper service.
        self._call_setbool(self.save_map_cli, "_slam_save_up", "save_map")

    def _restart_slam(self):
        # Trigger a slam restart through the System Manager API if reachable,
        # otherwise just log — the manager's watchdog/reconcile will recover.
        try:
            import urllib.request
            urllib.request.urlopen("http://127.0.0.1:8080/api/v1/control/restart?component=slam",
                                   timeout=3)
            self.get_logger().info("Requested slam restart via System Manager")
        except Exception as e:
            self.get_logger().warn(f"Could not request slam restart ({e}) — manual restart needed")

    def publish_status(self, event=None):
        payload = {
            "slam_state": -1,
            "slam_state_name": self.slam_state_name,
            "tracking_frequency": self.tracking_frequency,
            "num_keyframes": self.num_keyframes,
            "current_mode": self.current_mode,
            "initialized": self.initialized,
            "timestamp": self.get_clock().now().to_msg().sec,
        }
        if event:
            payload["event"] = event
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)
        try:
            with open("/tmp/terraslam_slam_status", "w") as f:
                f.write(msg.data)
        except Exception as e:
            self.get_logger().error(f"Status file write error: {e}")


def main():
    rclpy.init()
    node = SlamModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
