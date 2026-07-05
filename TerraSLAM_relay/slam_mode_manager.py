import rclpy
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Int8, String
from orb_slam3_ros2_wrapper.srv import MapControl
import json
import os

class SlamModeManager(Node):
    def __init__(self):
        super().__init__('slam_mode_manager')
        
        self.current_mode = "LOCALIZATION_ONLY"
        self.slam_state = -1
        self.slam_state_name = "UNKNOWN"
        self.initialized = False
        
        self.mode_file = "/tmp/terraslam_slam_mode"
        self.cmd_file = "/tmp/terraslam_slam_cmd"
        self.path_file = "/tmp/terraslam_slam_path"
        
        # Subscribers
        self.create_subscription(Int8, 'orb_slam3/tracking_state', self.state_cb, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/slam/mode_status', 10)
        self.drone_cmd_pub = self.create_publisher(String, '/drone/mode_command', 10)
        
        # Service client
        self.map_control_cli = self.create_client(MapControl, 'orb_slam3/map_control')
        
        self.get_logger().info("Waiting for orb_slam3/map_control service...")
        if not self.map_control_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service /orb_slam3/map_control not available!")
            return
        
        self.get_logger().info("Service connected. Starting in LOCALIZATION_ONLY...")
        self.send_map_control(3)                    # 3 = LOCALIZATION_ONLY
        
        self.create_timer(0.2, self.check_file_flags)   # 5 Hz вместо 10 Hz
        self.publish_status()

    def future_done(self, future: Future, command: int, filepath: str = ""):
        try:
            res = future.result()
            if res is None:
                self.get_logger().error(f"MapControl[{command}]: Service call failed (no response)")
                return False
            
            self.get_logger().info(f"MapControl[{command}]: success={res.success}, msg='{res.message}'")
            
            if res.success:
                if command == 3:
                    self.current_mode = "LOCALIZATION_ONLY"
                elif command == 4:
                    self.current_mode = "SLAM_MAPPING"
                self.publish_status(event="MODE_SWITCHED")
                return True
            else:
                self.get_logger().error(f"MapControl[{command}] rejected by ORB-SLAM3: {res.message}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"MapControl[{command}] exception: {e}")
            return False

    def send_map_control(self, command: int, filepath: str = "") -> bool:
        req = MapControl.Request()
        req.command = command
        req.filepath = filepath or ""
        
        future = self.map_control_cli.call_async(req)
        future.add_done_callback(lambda f: self.future_done(f, command, filepath))
        return True   # мы вернём настоящий результат позже

    def check_file_flags(self):
        # === Mode switch ===
        if os.path.exists(self.mode_file):
            try:
                with open(self.mode_file, "r") as f:
                    mode = f.read().strip()
                
                if mode == "LOCALIZATION_ONLY" and self.current_mode != "LOCALIZATION_ONLY":
                    self.get_logger().info("→ Switching to LOCALIZATION_ONLY")
                    self.send_map_control(3)
                
                elif mode == "SLAM_MAPPING" and self.current_mode != "SLAM_MAPPING":
                    self.get_logger().info("→ Switching to SLAM_MAPPING")
                    self.send_map_control(4)
                
                os.remove(self.mode_file)
            except Exception as e:
                self.get_logger().error(f"Mode file error: {e}")

        # === Direct commands (reset/save/load) ===
        if os.path.exists(self.cmd_file):
            try:
                with open(self.cmd_file, "r") as f:
                    cmd = f.read().strip()
                
                filepath = ""
                if os.path.exists(self.path_file):
                    with open(self.path_file, "r") as f:
                        filepath = f.read().strip()
                
                if cmd == "reset":
                    self.send_map_control(0)
                elif cmd == "save_map":
                    self.send_map_control(1, filepath or "/home/orb/Database/map.osa")
                elif cmd == "load_map":
                    self.send_map_control(2, filepath or "/home/orb/Database/map.osa")
                
                os.remove(self.cmd_file)
                if os.path.exists(self.path_file):
                    os.remove(self.path_file)
            except Exception as e:
                self.get_logger().error(f"Cmd file error: {e}")

    def state_cb(self, msg: Int8):
        self.slam_state = msg.data
        state_map = {0: "NO_IMAGES", 1: "NOT_INITIALIZED", 2: "OK", 3: "LOST"}
        self.slam_state_name = state_map.get(msg.data, "UNKNOWN")

        if msg.data == 2 and not self.initialized:
            self.initialized = True
            self.current_mode = "SLAM_MAPPING"
            self.get_logger().info("First tracking OK → switching to SLAM_MAPPING")
            self.send_map_control(4)

        self.publish_status()

    def publish_status(self, event=None):
        payload = {
            "slam_state": self.slam_state,
            "slam_state_name": self.slam_state_name,
            "current_mode": self.current_mode,
            "initialized": self.initialized,
            "timestamp": self.get_clock().now().to_msg().sec
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


if __name__ == '__main__':
    main()

