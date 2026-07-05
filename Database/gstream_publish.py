import cv2
import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class StreamPublisher(Node):
    def __init__(self, port=5600):
        super().__init__('stream_publisher')
        
        self.pub_raw = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_compressed = self.create_publisher(
            CompressedImage, 
            '/camera/image_raw/compressed', 
            10
        )
        self.bridge = CvBridge()
        
        # Build pipeline properly
        pipeline_str = (
            "udpsrc port={} ! ".format(port) +
            "application/x-rtp,media=video,encoding-name=H264 ! " +
            "rtph264depay ! h264parse ! avdec_h264 ! " +
            "videoconvert ! appsink sync=false"
        )
        
        self.get_logger().info(f'Pipeline: {pipeline_str}')
        self.get_logger().info(f'Opening stream on port {port}...')
        
        self.cap = cv2.VideoCapture(pipeline_str, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open stream!')
            raise RuntimeError('Cannot open GStreamer stream')
        
        self.get_logger().info('Stream opened successfully!')
        self.timer = self.create_timer(0.04, self.timer_callback)
        self.frame_count = 0

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to grab frame, reconnection may be needed')
            return
        
        stamp = self.get_clock().now().to_msg()
        
        # --- Publish RAW ---
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = stamp
            ros_image.header.frame_id = "camera"
            self.pub_raw.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing raw: {e}')
        
        # --- Publish COMPRESSED ---
        try:
            success, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if success:
                comp_msg = CompressedImage()
                comp_msg.header.stamp = stamp
                comp_msg.header.frame_id = "camera"
                comp_msg.format = "jpeg"
                comp_msg.data = buffer.tobytes()
                self.pub_compressed.publish(comp_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing compressed: {e}')
        
        self.frame_count += 1
        if self.frame_count % 25 == 0:
            self.get_logger().info(f'Publishing... frames: {self.frame_count}')

def main(args=None):
    rclpy.init(args=args)
    
    port = 5600
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    
    print(f"\n🚀 Stream Publisher - Port {port}")
    
    stream_publisher = StreamPublisher(port)
    try:
        rclpy.spin(stream_publisher)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    finally:
        if hasattr(stream_publisher, 'cap'):
            stream_publisher.cap.release()
        stream_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
