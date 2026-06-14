import os
import cv2
import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self, image_dir):
        super().__init__('image_publisher')
       # qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST,depth=10)
        self.pub_raw = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_compressed = self.create_publisher(
            CompressedImage, 
            '/camera/image_raw/compressed', 
            10
        )
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.04, self.timer_callback)
        self.image_files = sorted(os.listdir(image_dir), key=lambda x: float(x.split('.')[0]))
        self.image_dir = image_dir
        self.index = 0

    def timer_callback(self):
        if self.index < len(self.image_files):
            img_path = os.path.join(self.image_dir, self.image_files[self.index])
            cv_image = cv2.imread(img_path)
            if cv_image is not None:
                stamp = self.get_clock().now().to_msg()
                
                # --- 1. Публикуем RAW для SLAM ---
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                ros_image.header.stamp = stamp
                ros_image.header.frame_id = "camera"
                self.pub_raw.publish(ros_image)
                
                # --- 2. Публикуем COMPRESSED для браузера ---
                success, buffer = cv2.imencode('.jpg', cv_image)
                if success:
                    comp_msg = CompressedImage()
                    comp_msg.header.stamp = stamp
                    comp_msg.header.frame_id = "camera"
                    comp_msg.format = "jpeg"
                    comp_msg.data = buffer.tobytes()
                    self.pub_compressed.publish(comp_msg)
                
                self.get_logger().info(f'Publishing {self.image_files[self.index]}')
            
            self.index += 1
        else:
            self.index = 0
            
def main(args=None):
    rclpy.init(args=args)
    print("\n" + "="*60, file=sys.stderr)
    print("🚀 Image Publisher Started", file=sys.stderr)
    print(f"📦 sys.argv: {sys.argv}", file=sys.stderr)
    print(f"📂 Argument count: {len(sys.argv)}", file=sys.stderr)
    for i, arg in enumerate(sys.argv):
        print(f"   [{i}] '{arg}'", file=sys.stderr)
    if len(sys.argv) < 2:
        print("Usage: python main.py <path_to_images>")
        return
    image_dir = sys.argv[1]
    image_publisher = ImagePublisher(image_dir)
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
