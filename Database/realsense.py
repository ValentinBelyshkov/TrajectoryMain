#!/usr/bin/env python3
import cv2
import rclpy
import sys
import os
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("ERROR: pyrealsense2 not installed")


class RealSensePublisher(Node):
    def __init__(self, serial=None):
        super().__init__('realsense_publisher')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Паблишеры
        self.publisher = self.create_publisher(Image, '/camera/image_raw', qos)
        self.publisher_compressed = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', qos
        )
        
        # Подписка на позу камеры (ORB-SLAM3)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/camera_pose', self.pose_callback, qos
        )
        self.current_pose = None  # x, y, z
        
        self.bridge = CvBridge()
        
        # Создаём папку new_frames рядом со скриптом
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.frames_dir = os.path.join(script_dir, 'new_frames')
        os.makedirs(self.frames_dir, exist_ok=True)
        self.get_logger().info(f'=== FRAMES DIR: {self.frames_dir} ===')
        self.get_logger().info(f'=== SCRIPT DIR: {script_dir} ===')
        
        if not REALSENSE_AVAILABLE:
            raise RuntimeError('RealSense SDK not available')
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        
        if serial:
            self.config.enable_device(serial)
        
        self.get_logger().info('Starting RealSense pipeline...')
        self.pipeline.start(self.config)
        self.get_logger().info('RealSense pipeline started')
        
        for i in range(10):
            self.pipeline.wait_for_frames()
        
        self.timer = self.create_timer(1/30, self.timer_callback)
        self.frame_count = 0
        self.saved_count = 0
        self._shutdown_requested = False
        
    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.current_pose = (x, y, z)
        
    def timer_callback(self):
        if self._shutdown_requested:
            return
            
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            
            if not color_frame:
                self.get_logger().warn('=== NO COLOR FRAME ===')
                return
            
            color_image = np.asanyarray(color_frame.get_data())
            color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            if color_image.size == 0:
                self.get_logger().warn('=== EMPTY IMAGE ===')
                return
            
            stamp = self.get_clock().now().to_msg()
            frame_id = 'camera'
            
            # --- Публикация raw изображения ---
            ros_image = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            ros_image.header.stamp = stamp
            ros_image.header.frame_id = frame_id
            self.publisher.publish(ros_image)
            
            # --- Публикация сжатого изображения ---
            success, encoded_image = cv2.imencode('.jpg', color_image)
            if success:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = stamp
                compressed_msg.header.frame_id = frame_id
                compressed_msg.format = "jpeg"
                compressed_msg.data = encoded_image.tobytes()
                self.publisher_compressed.publish(compressed_msg)
            
            # --- Сохранение кадра и позы ---
            self.get_logger().info(
                f'=== FRAME {self.frame_count:04d} | pose={self.current_pose} ==='
            )
            
            if self.current_pose is not None:
                self.save_frame(color_image)
            else:
                self.get_logger().warn(
                    f'=== SKIPPING frame {self.frame_count:04d}: NO POSE DATA ==='
                )
            
            self.frame_count += 1
            if self.frame_count % 300 == 0:
                self.get_logger().info(
                    f'=== Published {self.frame_count} frames, saved {self.saved_count} ==='
                )
                
        except Exception as e:
            if not self._shutdown_requested:
                self.get_logger().warn(f'=== Frame error: {e} ===')
    
    def save_frame(self, color_image):
        filename_base = f'frame_{self.frame_count:04d}'
        jpg_path = os.path.join(self.frames_dir, f'{filename_base}.jpg')
        txt_path = os.path.join(self.frames_dir, f'{filename_base}.txt')
        
        self.get_logger().info(f'=== SAVING: {jpg_path} ===')
        
        # Проверяем что изображение валидно
        self.get_logger().info(
            f'=== IMAGE shape={color_image.shape}, dtype={color_image.dtype} ==='
        )
        
        # Сохраняем JPEG
        ok = cv2.imwrite(jpg_path, color_image)
        self.get_logger().info(f'=== cv2.imwrite returned: {ok} ===')
        
        if not ok:
            self.get_logger().error(f'=== FAILED to save {jpg_path} ===')
            return
        
        # Сохраняем позу в txt
        x, y, z = self.current_pose
        with open(txt_path, 'w') as f:
            f.write(f'{x:.6f} {y:.6f} {z:.6f}\n')
        
        self.saved_count += 1
        self.get_logger().info(
            f'=== OK: saved {filename_base}.jpg + .txt '
            f'(total saved: {self.saved_count}) ==='
        )
    
    def shutdown(self):
        self._shutdown_requested = True
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()
        if hasattr(self, 'pipeline') and self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass


def main(args=None):
    if not REALSENSE_AVAILABLE:
        print("ERROR: pyrealsense2 not installed")
        return 1
    
    rclpy.init(args=args)
    serial = sys.argv[1] if len(sys.argv) > 1 else None
    
    publisher = None
    executor = None
    
    try:
        print(" Starting RealSense Publisher...")
        publisher = RealSensePublisher(serial)
        print(" Node ready. Press Ctrl+C to exit.")
        
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(publisher)
        executor.spin()
        
    except KeyboardInterrupt:
        print("\n Interrupted by user")
    except Exception as e:
        print(f"\n Error: {e}")
        return 1
    finally:
        if publisher is not None:
            try:
                publisher.shutdown()
            except Exception:
                pass
            try:
                publisher.destroy_node()
            except Exception:
                pass
        
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
                
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
