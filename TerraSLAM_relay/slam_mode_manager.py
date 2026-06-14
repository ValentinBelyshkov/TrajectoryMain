#!/usr/bin/env python3
"""
slam_mode_manager.py — ROS2 node for SLAM mode control.
Reads commands from file flags, publishes status to /slam/mode_status.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from orb_slam3_ros2_wrapper.srv import MapControl
import json
import os
import time

class SlamModeManager(Node):
    def __init__(self):
        super().__init__('slam_mode_manager')
        
        # State
        self.initialized = False
        self.current_mode = "LOCALIZATION_ONLY"
        self.slam_state = -1
        self.slam_state_name = "UNKNOWN"
        
        # File flags
        self.mode_file = "/tmp/terraslam_slam_mode"
        self.cmd_file = "/tmp/terraslam_slam_cmd"
        self.path_file = "/tmp/terraslam_slam_path"
        
        # ROS2 subscribers
        self.create_subscription(Int8, 'orb_slam3/tracking_state', self.state_cb, 10)
        
        # ROS2 publishers
        self.status_pub = self.create_publisher(String, '/slam/mode_status', 10)
        self.drone_cmd_pub = self.create_publisher(String, '/drone/mode_command', 10)
        
        # MapControl service client
        self.map_control_cli = self.create_client(MapControl, 'orb_slam3/map_control')
        
        self.get_logger().info("Waiting for /orb_slam3/map_control...")
        while not self.map_control_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Still waiting...")
        
        # Start in localization mode
        self.send_map_control(3)
        self.get_logger().info("Started in LOCALIZATION_ONLY mode")
        
        # Timer for checking file flags (10 Hz)
        self.create_timer(0.1, self.check_file_flags)
        
        # Publish initial status
        self.publish_status()
    
    def send_map_control(self, command: int, filepath: str = "") -> bool:
        req = MapControl.Request()
        req.command = command
        req.filepath = filepath
        
        future = self.map_control_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result():
            res = future.result()
            self.get_logger().info(f"MapControl[{command}]: {res.message}")
            return res.success
        self.get_logger().error(f"MapControl[{command}]: failed")
        return False
    
    def check_file_flags(self):
        """Check for commands from TerraSLAM Manager (file-based IPC)."""
        # Check mode switch
        if os.path.exists(self.mode_file):
            try:
                with open(self.mode_file, "r") as f:
                    mode = f.read().strip()
                
                if mode == "LOCALIZATION_ONLY" and self.current_mode != "LOCALIZATION_ONLY":
                    if self.send_map_control(3):
                        self.current_mode = "LOCALIZATION_ONLY"
                        self.publish_status(event="MODE_SWITCHED")
                        self.get_logger().info("Switched to LOCALIZATION_ONLY")
                
                elif mode == "SLAM_MAPPING" and self.current_mode != "SLAM_MAPPING":
                    if self.send_map_control(4):
                        self.current_mode = "SLAM_MAPPING"
                        self.publish_status(event="MODE_SWITCHED")
                        self.get_logger().info("Switched to SLAM_MAPPING")
                
                # Clean up
                os.remove(self.mode_file)
            except Exception as e:
                self.get_logger().error(f"Mode file error: {e}")
        
        # Check commands
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
                    self.get_logger().info("Reset command executed")
                
                elif cmd == "save_map":
                    self.send_map_control(1, filepath or "/home/orb/Database/map.osa")
                    self.get_logger().info(f"Save map: {filepath}")
                
                elif cmd == "load_map":
                    self.send_map_control(2, filepath or "/home/orb/Database/map.osa")
                    self.get_logger().info(f"Load map: {filepath}")
                
                # Clean up
                os.remove(self.cmd_file)
                if os.path.exists(self.path_file):
                    os.remove(self.path_file)
            except Exception as e:
                self.get_logger().error(f"Cmd file error: {e}")
    
    def state_cb(self, msg: Int8):
        self.slam_state = msg.data
        state_map = {0: "NO_IMAGES", 1: "NOT_INITIALIZED", 2: "OK", 3: "LOST"}
        self.slam_state_name = state_map.get(msg.data, "UNKNOWN")
        
        # Auto-switch on first tracking OK
        if self.slam_state == 2 and not self.initialized:
            self.initialized = True
            self.current_mode = "SLAM_MAPPING"
            self.get_logger().warn("🎯 FIRST TRACKING_OK! Auto-switching to SLAM+Mapping")
            self.send_map_control(4)
            self.publish_status(event="AUTO_MODE_SWITCHED")
        
        elif self.slam_state == 3:
            self.publish_status(event="TRACKING_LOST")
        else:
            self.publish_status()
    
    def publish_status(self, event=None):
        payload = {
            "slam_state": self.slam_state,
            "slam_state_name": self.slam_state_name,
            "drone_mode": self.current_mode,
            "initialized": self.initialized,
            "timestamp": self.get_clock().now().to_msg().sec
        }
        if event:
            payload["event"] = event
        
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

def main():
    rclpy.init()
    node = SlamModeManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
