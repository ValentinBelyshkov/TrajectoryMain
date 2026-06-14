import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import socket
import struct
import cv2
import numpy as np
import threading
import time
import json
import math
from pathlib import Path
from pymap3d import enu2geodetic
from transform_utils import transform_point, read_transform_matrix

class SLAM2GPS:
    """Transform SLAM coordinates to LLA using config file."""
    def __init__(self, jpath):
        cfg = json.loads(Path(jpath).read_text())
        self.s, cfg_r, cfg_t = cfg["scale"], np.array(cfg["rotation"]), np.array(cfg["translation"])
        self.R, self.t = cfg_r, cfg_t.reshape(3,1)
        self.lat0, self.lon0, self.alt0 = cfg["lat0"], cfg["lon0"], cfg["alt0"]

    def slam_to_lla(self, xyz):
        xyz = np.asarray(xyz).reshape(3,-1)
        enu = (self.s*self.R@xyz+self.t).T
        return self.enu_to_lla(enu[:,0], enu[:,1], enu[:,2])

    def enu_to_lla(self, E, N, U):
        lat, lon, alt = enu2geodetic(E, N, U, self.lat0, self.lon0, self.alt0)
        return np.c_[lat, lon, alt]

class ImageRelay(Node):
    def __init__(self):
        super().__init__('image_relay')

        # Initialize ROS2 publishers and subscribers
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        # NEW: GPS Publisher
        self.gps_publisher_ = self.create_publisher(
            NavSatFix, 
            '/camera/gps', 
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )
        )

        self.subscription = self.create_subscription(
            Pose,
            '/camera_pose',
            self.pose_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            ))

        self.bridge = CvBridge()

        # Initialize camera pose
        self.current_pose = None
        self.current_gps = None  # Store last GPS
        self.pose_lock = threading.Lock()

        # Initialize TCP server
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', 43322))
        self.server_socket.listen(1)
        self.get_logger().info('TCP server started on port 43322')

        # Initialize GPS conversion
        self.transform_json = "/home/orb/colcon_ws/src/orb_slam3_ros2_wrapper/transform.json"
        self.slam_transform_file = "/home/orb/colcon_ws/src/orb_slam3_ros2_wrapper/slam_transform_matrix.txt"

        try:
            self.slam2gps = SLAM2GPS(self.transform_json)
            self.base_transform_matrix = read_transform_matrix(self.slam_transform_file)
            self.get_logger().info('✅ GPS conversion initialized')
            self.get_logger().info(f'   Reference: {self.slam2gps.lat0}, {self.slam2gps.lon0}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to initialize GPS conversion: {e}')
            self.slam2gps = None
            self.base_transform_matrix = None

        # Start TCP server in a separate thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def pose_callback(self, msg):
        """Handle new pose, convert to GPS, and publish."""
        with self.pose_lock:
            self.current_pose = msg

            # Convert to GPS immediately
            if self.slam2gps is not None and self.base_transform_matrix is not None:
                try:
                    x, y, z = msg.position.x, msg.position.y, msg.position.z

                    # Check for special status codes from ORB-SLAM3
                    if x == -3.0 and y == -3.0 and z == -3.0:
                        self.get_logger().warn('Tracking lost')
                        return
                    if x == -1.0 and y == -1.0 and z == -1.0:
                        self.get_logger().warn('Not initialized')
                        return
                    if x == 0.0 and y == 0.0 and z == 0.0:
                        self.get_logger().warn('No images yet')
                        return

                    # Transform using base matrix (hand-eye calibration)
                    x_t, y_t, z_t = transform_point(self.base_transform_matrix, [x, y, z])

                    # Convert to GPS coordinates
                    lat_lon_alt = self.slam2gps.slam_to_lla([x_t, y_t, z_t])
                    lat, lon, alt = float(lat_lon_alt[0][0]), float(lat_lon_alt[0][1]), float(lat_lon_alt[0][2])

                    # Store current GPS
                    self.current_gps = (lat, lon, alt)

                    # Publish NavSatFix message
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = self.get_clock().now().to_msg()
                    gps_msg.header.frame_id = "camera"
                    gps_msg.status.status = NavSatStatus.STATUS_FIX
                    gps_msg.status.service = NavSatStatus.SERVICE_GPS
                    gps_msg.latitude = lat
                    gps_msg.longitude = lon
                    gps_msg.altitude = alt

                    # Set covariance (optional - set high if uncertain)
                    gps_msg.position_covariance = [0.0] * 9
                    gps_msg.position_covariance[0] = 0.01  # x variance (m^2)
                    gps_msg.position_covariance[4] = 0.01  # y variance
                    gps_msg.position_covariance[8] = 0.04  # z variance (height less accurate)
                    gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

                    self.gps_publisher_.publish(gps_msg)
                    self.get_logger().info(f'📍 GPS: {lat:.7f}, {lon:.7f}, {alt:.2f}m')

                except Exception as e:
                    self.get_logger().error(f'GPS conversion failed: {e}')
            else:
                self.get_logger().warn('GPS conversion not available')

    def send_pose(self, client_socket):
        """Send pose/GPS back to TCP client."""
        with self.pose_lock:
            try:
                if self.current_gps is not None:
                    # Send GPS coordinates (lat, lon, alt)
                    lat, lon, alt = self.current_gps
                    pose_data = struct.pack('3d', lat, lon, alt)
                    self.get_logger().debug(f'Sent GPS to client: {lat:.6f}, {lon:.6f}')
                elif self.current_pose is not None:
                    # Fallback to raw pose if GPS not available
                    x = self.current_pose.position.x
                    y = self.current_pose.position.y
                    z = self.current_pose.position.z

                    # Check status codes
                    if (x == -3.0 and y == -3.0 and z == -3.0):
                        pose_data = struct.pack('3d', -3.0, -3.0, -3.0)
                        self.get_logger().info('Sent tracking lost status')
                    elif (x == -1.0 and y == -1.0 and z == -1.0):
                        pose_data = struct.pack('3d', -1.0, -1.0, -1.0)
                        self.get_logger().info('Sent not initialized status')
                    elif (x == 0.0 and y == 0.0 and z == 0.0):
                        pose_data = struct.pack('3d', 0.0, 0.0, 0.0)
                        self.get_logger().info('Sent no images status')
                    else:
                        pose_data = struct.pack('3d', x, y, z)

                    client_socket.sendall(pose_data)
                else:
                    # Initializing
                    pose_data = struct.pack('3d', 0.0, 0.0, 0.0)
                    client_socket.sendall(pose_data)

            except Exception as e:
                self.get_logger().error(f'Failed to send pose: {e}')

    def handle_client(self, client_socket):
        try:
            while True:
                # Receive image size
                size_data = client_socket.recv(4)
                if not size_data:
                    self.get_logger().info('Client disconnected')
                    break

                image_size = struct.unpack('!I', size_data)[0]

                # Receive image data
                received_data = b''
                while len(received_data) < image_size:
                    chunk = client_socket.recv(min(4096, image_size - len(received_data)))
                    if not chunk:
                        break
                    received_data += chunk

                if len(received_data) != image_size:
                    continue

                # Decode and publish
                try:
                    nparr = np.frombuffer(received_data, np.uint8)
                    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                    if image is not None:
                        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                        self.publisher_.publish(msg)
                        self.send_pose(client_socket)

                except Exception as e:
                    self.get_logger().error(f'Error processing image: {e}')

        except Exception as e:
            self.get_logger().error(f'Client error: {e}')
        finally:
            client_socket.close()

    def run_server(self):
        try:
            while True:
                client_socket, address = self.server_socket.accept()
                self.get_logger().info(f'New connection from {address}')
                client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
                client_thread.daemon = True
                client_thread.start()
        except Exception as e:
            self.get_logger().error(f'Server error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

