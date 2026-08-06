"""Publish frames from a directory to /camera/image_raw for ORB-SLAM3.

This node feeds ORB-SLAM3 with images from disk. It must publish real ROS
messages -- an earlier revision replaced this with a plain `cv2.imshow` viewer,
which silently starved SLAM of input (no frames published => nothing tracked =>
nothing saved into procframe and an empty .osa map).

Frames are re-published in a loop until the process is stopped, because
ORB-SLAM3 tracks far slower than the publish rate and the orchestrator decides
when the run is over.
"""
import argparse
import os
import re
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


def _frame_index(filename: str) -> int:
    """Extract the leading numeric index from a frame filename.

    Handles both `0001.jpg` (video-extracted) and `frame_0001.jpg`
    (camera-recorded) naming conventions. Falls back to 0 so an unexpected
    name can never crash the sort (the old `float(x.split('.')[0])` did).
    """
    stem = os.path.splitext(os.path.basename(filename))[0]
    match = re.search(r"(\d+)", stem)
    return int(match.group(1)) if match else 0


class ImagePublisher(Node):
    def __init__(self, image_dir: str, fps: float, loop: bool, procframe_dir: str = ""):
        super().__init__("image_publisher")

        self.pub_raw = self.create_publisher(Image, "/camera/image_raw", 10)
        self.pub_compressed = self.create_publisher(
            CompressedImage, "/camera/image_raw/compressed", 10
        )
        self.bridge = CvBridge()

        self.image_dir = image_dir
        self.loop = loop
        self.procframe_dir = procframe_dir
        self.index = 0
        self.published = 0
        self.failed = 0
        self._last_report = time.time()

        self.image_files = sorted(
            [
                f
                for f in os.listdir(image_dir)
                if f.lower().endswith((".jpg", ".jpeg", ".png"))
            ],
            key=_frame_index,
        )

        # procframe is written by ORB-SLAM3 itself; make sure it exists and is
        # writable so a permission problem surfaces here instead of as a silent
        # "no frames were saved" at the end of the run.
        if procframe_dir:
            try:
                os.makedirs(procframe_dir, exist_ok=True)
                probe = os.path.join(procframe_dir, ".write_test")
                with open(probe, "w") as fh:
                    fh.write("ok")
                os.remove(probe)
                self.get_logger().info(f"procframe dir OK (writable): {procframe_dir}")
            except OSError as exc:
                self.get_logger().error(
                    f"procframe dir NOT writable: {procframe_dir} ({exc})"
                )

        self.get_logger().info(
            f"Image publisher: dir={image_dir} frames={len(self.image_files)} "
            f"fps={fps} loop={loop}"
        )
        if not self.image_files:
            self.get_logger().error(f"No images found in {image_dir} -- nothing to publish")

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def timer_callback(self):
        if not self.image_files:
            return

        if self.index >= len(self.image_files):
            if not self.loop:
                self.get_logger().info(
                    f"Published all {self.published} frames once; idling "
                    "(orchestrator will stop this process)"
                )
                self.timer.cancel()
                return
            self.get_logger().info(
                f"Reached end of {len(self.image_files)} frames, looping "
                f"(total published={self.published})"
            )
            self.index = 0

        name = self.image_files[self.index]
        img_path = os.path.join(self.image_dir, name)
        cv_image = cv2.imread(img_path)
        self.index += 1

        if cv_image is None:
            self.failed += 1
            self.get_logger().warn(f"Failed to read image: {img_path}")
            return

        stamp = self.get_clock().now().to_msg()

        # RAW for SLAM.
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        ros_image.header.stamp = stamp
        ros_image.header.frame_id = "camera"
        self.pub_raw.publish(ros_image)

        # COMPRESSED for the browser preview.
        ok, buffer = cv2.imencode(".jpg", cv_image)
        if ok:
            comp = CompressedImage()
            comp.header.stamp = stamp
            comp.header.frame_id = "camera"
            comp.format = "jpeg"
            comp.data = buffer.tobytes()
            self.pub_compressed.publish(comp)

        self.published += 1

        # Heartbeat once a second: proves frames really go out, and shows how
        # many procframe results ORB-SLAM3 has produced so far.
        now = time.time()
        if now - self._last_report >= 1.0:
            self._last_report = now
            saved = -1
            if self.procframe_dir:
                try:
                    saved = len(
                        [
                            f
                            for f in os.listdir(self.procframe_dir)
                            if f.lower().endswith(".jpg")
                        ]
                    )
                except OSError:
                    saved = -1
            subs = self.pub_raw.get_subscription_count()
            self.get_logger().info(
                f"published={self.published} failed={self.failed} "
                f"last={name} subscribers={subs} procframe_jpgs={saved}"
            )
            if subs == 0:
                self.get_logger().warn(
                    "No subscribers on /camera/image_raw -- ORB-SLAM3 is not "
                    "receiving frames (is the slam component running?)"
                )


def main(args=None):
    parser = argparse.ArgumentParser(description="Publish images from a directory to ROS")
    parser.add_argument("image_dir", help="Path to directory with images")
    parser.add_argument("--fps", type=float, default=25.0, help="Publish rate (default: 25)")
    parser.add_argument(
        "--procframe-dir",
        default="",
        help="Per-project procframe dir (checked for writability and monitored)",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Publish the sequence once instead of looping",
    )
    # ROS injects `__node:=...` style args; drop them before argparse.
    raw_args = [a for a in sys.argv[1:] if not a.startswith("__")]
    parsed = parser.parse_args(raw_args)

    if not os.path.isdir(parsed.image_dir):
        print(f"Image dir does not exist: {parsed.image_dir}", file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = ImagePublisher(
        parsed.image_dir,
        fps=parsed.fps,
        loop=not parsed.once,
        procframe_dir=parsed.procframe_dir,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"Shutting down: published={node.published} failed={node.failed}"
        )
        node.destroy_node()
        # SIGINT can make rclpy shut the context down on its own; calling
        # shutdown() again would raise and mask the real exit reason.
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
