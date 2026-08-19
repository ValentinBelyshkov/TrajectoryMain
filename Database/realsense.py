#!/usr/bin/env python3

import os
import sys
import threading
import queue
import time

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("ERROR: pyrealsense2 not installed")


class RealSensePublisher(Node):
    """
    RealSense -> ROS2 publisher.

    Основные принципы:
      1. Получение кадров RealSense выполняется в отдельном потоке.
      2. ROS timer НЕ используется для чтения камеры.
      3. JPEG compression выполняется отдельно.
      4. Сохранение JPEG выполняется отдельно.
      5. Timestamp берётся из timestamp самого кадра RealSense.
      6. Для камеры используется BEST_EFFORT + небольшой queue.
      7. При перегрузке старые кадры отбрасываются, а не накапливаются.
    """

    def __init__(
        self,
        serial=None,
        frames_dir=None,
        no_save_frames=False,
        enable_compressed=True,
    ):
        super().__init__("realsense_publisher")

        self._shutdown_requested = threading.Event()

        # ------------------------------------------------------------------
        # QoS для realtime camera stream
        # ------------------------------------------------------------------

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            durability=DurabilityPolicy.VOLATILE,
        )

        compressed_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            durability=DurabilityPolicy.VOLATILE,
        )

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------

        self.publisher = self.create_publisher(
            Image,
            "/cam0/image_raw",
            camera_qos,
        )

        self.enable_compressed = enable_compressed

        if self.enable_compressed:
            self.publisher_compressed = self.create_publisher(
                CompressedImage,
                "/camera/image_raw/compressed",
                compressed_qos,
            )
        else:
            self.publisher_compressed = None

        # ------------------------------------------------------------------
        # Pose subscriber
        # ------------------------------------------------------------------

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/orb_slam3/robot_pose_slam",
            self.pose_callback,
            pose_qos,
        )

        self.current_pose = None
        self.pose_lock = threading.Lock()

        # ------------------------------------------------------------------
        # CvBridge
        # ------------------------------------------------------------------

        self.bridge = CvBridge()

        # ------------------------------------------------------------------
        # Saving configuration
        # ------------------------------------------------------------------

        script_dir = os.path.dirname(os.path.abspath(__file__))

        self.save_frames = not no_save_frames

        if self.save_frames:
            self.frames_dir = (
                frames_dir
                if frames_dir
                else os.path.join(script_dir, "new_frames")
            )

            os.makedirs(self.frames_dir, exist_ok=True)

            self.get_logger().info(
                f"=== FRAMES DIR: {self.frames_dir} ==="
            )
        else:
            self.frames_dir = None

            self.get_logger().info(
                "=== FRAME SAVING DISABLED ==="
            )

        self.get_logger().info(
            f"=== SCRIPT DIR: {script_dir} ==="
        )

        # ------------------------------------------------------------------
        # RealSense availability
        # ------------------------------------------------------------------

        if not REALSENSE_AVAILABLE:
            raise RuntimeError(
                "RealSense SDK not available"
            )

        # ------------------------------------------------------------------
        # RealSense pipeline
        # ------------------------------------------------------------------

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # IMPORTANT:
        # RGB8 -> публикуем напрямую как RGB.
        #
        # ORB-SLAM config:
        # Camera.RGB: 1
        #
        self.config.enable_stream(
            rs.stream.color,
            640,
            480,
            rs.format.rgb8,
            30,
        )

        if serial:
            self.config.enable_device(serial)

        self.get_logger().info(
            "Starting RealSense pipeline..."
        )

        profile = self.pipeline.start(self.config)

        self.get_logger().info(
            "RealSense pipeline started"
        )

        # ------------------------------------------------------------------
        # Warm-up
        # ------------------------------------------------------------------

        self.get_logger().info(
            "Warming up RealSense..."
        )

        for _ in range(10):
            try:
                self.pipeline.wait_for_frames(timeout_ms=1000)
            except RuntimeError:
                pass

        self.get_logger().info(
            "RealSense warm-up complete"
        )

        # ------------------------------------------------------------------
        # Counters
        # ------------------------------------------------------------------

        self.frame_count = 0
        self.published_count = 0
        self.dropped_save_count = 0
        self.dropped_compressed_count = 0
        self.saved_count = 0

        self.counter_lock = threading.Lock()

        # ------------------------------------------------------------------
        # Timestamp synchronization
        # ------------------------------------------------------------------

        self._timestamp_lock = threading.Lock()

        self._rs_start_timestamp_ms = None
        self._ros_start_timestamp_ns = None

        # ------------------------------------------------------------------
        # Queues
        # ------------------------------------------------------------------

        # Небольшая очередь сохранения.
        # Если диск не успевает — старые кадры будут отбрасываться.
        self.save_queue = queue.Queue(maxsize=30)

        # Отдельная очередь для JPEG compression.
        if self.enable_compressed:
            self.compressed_queue = queue.Queue(maxsize=5)
        else:
            self.compressed_queue = None

        # ------------------------------------------------------------------
        # Threads
        # ------------------------------------------------------------------

        self.capture_thread = threading.Thread(
            target=self.capture_loop,
            name="realsense_capture",
            daemon=True,
        )

        self.capture_thread.start()

        if self.save_frames:
            self.save_thread = threading.Thread(
                target=self.save_loop,
                name="frame_saver",
                daemon=True,
            )

            self.save_thread.start()
        else:
            self.save_thread = None

        if self.enable_compressed:
            self.compressed_thread = threading.Thread(
                target=self.compressed_loop,
                name="jpeg_publisher",
                daemon=True,
            )

            self.compressed_thread.start()
        else:
            self.compressed_thread = None

        # ------------------------------------------------------------------
        # Statistics thread
        # ------------------------------------------------------------------

        self.statistics_thread = threading.Thread(
            target=self.statistics_loop,
            name="statistics",
            daemon=True,
        )

        self.statistics_thread.start()

    # ======================================================================
    # Pose
    # ======================================================================

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        with self.pose_lock:
            self.current_pose = (x, y, z)

    # ======================================================================
    # RealSense timestamp -> ROS timestamp
    # ======================================================================

    def realtime_stamp_from_frame(self, color_frame):
        """
        Преобразует timestamp RealSense в ROS timestamp.

        Важно:
        header.stamp теперь зависит от времени захвата кадра,
        а не от того, когда callback успел его обработать.
        """

        rs_timestamp_ms = color_frame.get_timestamp()

        with self._timestamp_lock:

            if self._rs_start_timestamp_ms is None:
                self._rs_start_timestamp_ms = rs_timestamp_ms

                self._ros_start_timestamp_ns = (
                    self.get_clock().now().nanoseconds
                )

            elapsed_ms = (
                rs_timestamp_ms
                - self._rs_start_timestamp_ms
            )

            stamp_ns = (
                self._ros_start_timestamp_ns
                + int(elapsed_ms * 1_000_000.0)
            )

        sec = stamp_ns // 1_000_000_000
        nanosec = stamp_ns % 1_000_000_000

        return sec, nanosec

    # ======================================================================
    # Capture loop
    # ======================================================================

    def capture_loop(self):
        """
        Основной realtime loop.

        НИКАКОГО cv2.imwrite().
        НИКАКОГО cv2.imencode().
        НИКАКИХ тяжёлых операций.

        Только:
          RealSense -> numpy -> ROS publish
        """

        self.get_logger().info(
            "=== Capture thread started ==="
        )

        while not self._shutdown_requested.is_set():

            try:
                frames = self.pipeline.wait_for_frames(
                    timeout_ms=1000
                )

                color_frame = frames.get_color_frame()

                if not color_frame:
                    self.get_logger().warn(
                        "=== NO COLOR FRAME ==="
                    )
                    continue

                # ----------------------------------------------------------
                # Получаем RGB image напрямую.
                #
                # НИКАКОГО cvtColor()
                # ----------------------------------------------------------

                color_image = np.asanyarray(
                    color_frame.get_data()
                )

                if color_image is None:
                    continue

                if color_image.size == 0:
                    self.get_logger().warn(
                        "=== EMPTY IMAGE ==="
                    )
                    continue

                # ----------------------------------------------------------
                # Timestamp из RealSense
                # ----------------------------------------------------------

                sec, nanosec = self.realtime_stamp_from_frame(
                    color_frame
                )

                # ----------------------------------------------------------
                # ROS Image
                # ----------------------------------------------------------

                try:
                    ros_image = self.bridge.cv2_to_imgmsg(
                        color_image,
                        encoding="rgb8",
                    )
                except Exception as e:
                    self.get_logger().error(
                        f"cv_bridge error: {e}"
                    )
                    continue

                ros_image.header.stamp.sec = sec
                ros_image.header.stamp.nanosec = nanosec
                ros_image.header.frame_id = "camera"

                # ----------------------------------------------------------
                # Publish raw image
                # ----------------------------------------------------------

                self.publisher.publish(ros_image)

                with self.counter_lock:
                    self.frame_count += 1
                    self.published_count += 1

                    current_frame_id = self.frame_count

                # ----------------------------------------------------------
                # Для worker threads нужна копия.
                # ----------------------------------------------------------

                frame_copy = color_image.copy()

                # ----------------------------------------------------------
                # JPEG queue
                # ----------------------------------------------------------

                if self.enable_compressed:

                    compressed_item = (
                        frame_copy,
                        sec,
                        nanosec,
                    )

                    try:
                        self.compressed_queue.put_nowait(
                            compressed_item
                        )

                    except queue.Full:
                        with self.counter_lock:
                            self.dropped_compressed_count += 1

                # ----------------------------------------------------------
                # Save queue
                # ----------------------------------------------------------

                if self.save_frames:

                    save_item = (
                        current_frame_id,
                        frame_copy,
                        sec,
                        nanosec,
                    )

                    try:
                        self.save_queue.put_nowait(
                            save_item
                        )

                    except queue.Full:
                        with self.counter_lock:
                            self.dropped_save_count += 1

            except RuntimeError as e:

                if self._shutdown_requested.is_set():
                    break

                self.get_logger().warn(
                    f"RealSense runtime error: {e}"
                )

            except Exception as e:

                if self._shutdown_requested.is_set():
                    break

                self.get_logger().error(
                    f"=== Capture error: {e} ==="
                )

        self.get_logger().info(
            "=== Capture thread stopped ==="
        )

    # ======================================================================
    # JPEG compression worker
    # ======================================================================

    def compressed_loop(self):
        self.get_logger().info(
            "=== JPEG publisher thread started ==="
        )

        while not self._shutdown_requested.is_set():

            try:
                item = self.compressed_queue.get(
                    timeout=0.5
                )

            except queue.Empty:
                continue

            try:
                (
                    color_image,
                    sec,
                    nanosec,
                ) = item

                success, encoded_image = cv2.imencode(
                    ".jpg",
                    color_image,
                    [
                        int(cv2.IMWRITE_JPEG_QUALITY),
                        85,
                    ],
                )

                if not success:
                    continue

                msg = CompressedImage()

                msg.header.stamp.sec = sec
                msg.header.stamp.nanosec = nanosec
                msg.header.frame_id = "camera"

                msg.format = "jpeg"

                msg.data = encoded_image.tobytes()

                self.publisher_compressed.publish(msg)

            except Exception as e:

                if not self._shutdown_requested.is_set():
                    self.get_logger().warn(
                        f"JPEG processing error: {e}"
                    )

            finally:
                self.compressed_queue.task_done()

        self.get_logger().info(
            "=== JPEG publisher thread stopped ==="
        )

    # ======================================================================
    # Save worker
    # ======================================================================

    def save_loop(self):
        self.get_logger().info(
            "=== Save thread started ==="
        )

        while not self._shutdown_requested.is_set():

            try:
                item = self.save_queue.get(
                    timeout=0.5
                )

            except queue.Empty:
                continue

            try:
                (
                    frame_number,
                    color_image,
                    sec,
                    nanosec,
                ) = item

                filename = (
                    f"frame_{frame_number:08d}.jpg"
                )

                path = os.path.join(
                    self.frames_dir,
                    filename,
                )

                ok = cv2.imwrite(
                    path,
                    color_image,
                    [
                        int(cv2.IMWRITE_JPEG_QUALITY),
                        95,
                    ],
                )

                if ok:

                    with self.counter_lock:
                        self.saved_count += 1

                else:

                    self.get_logger().error(
                        f"Failed to save: {path}"
                    )

            except Exception as e:

                if not self._shutdown_requested.is_set():
                    self.get_logger().error(
                        f"Save error: {e}"
                    )

            finally:
                self.save_queue.task_done()

        self.get_logger().info(
            "=== Save thread stopped ==="
        )

    # ======================================================================
    # Statistics
    # ======================================================================

    def statistics_loop(self):

        last_count = 0
        last_time = time.monotonic()

        while not self._shutdown_requested.wait(5.0):

            now = time.monotonic()

            with self.counter_lock:
                count = self.published_count
                saved = self.saved_count
                dropped_save = self.dropped_save_count
                dropped_compressed = (
                    self.dropped_compressed_count
                )

            dt = now - last_time
            df = count - last_count

            if dt > 0:
                fps = df / dt
            else:
                fps = 0.0

            self.get_logger().info(
                "=== Camera statistics: "
                f"FPS={fps:.2f}, "
                f"published={count}, "
                f"saved={saved}, "
                f"save_dropped={dropped_save}, "
                f"compressed_dropped={dropped_compressed} ==="
            )

            last_count = count
            last_time = now

    # ======================================================================
    # Shutdown
    # ======================================================================

    def shutdown(self):

        if self._shutdown_requested.is_set():
            return

        self.get_logger().info(
            "=== Shutting down RealSense publisher ==="
        )

        self._shutdown_requested.set()

        # --------------------------------------------------------------
        # Stop RealSense first
        # --------------------------------------------------------------

        if hasattr(self, "pipeline") and self.pipeline:

            try:
                self.pipeline.stop()

            except Exception:
                pass

        # --------------------------------------------------------------
        # Threads
        # --------------------------------------------------------------

        current_thread = threading.current_thread()

        threads = [
            getattr(self, "capture_thread", None),
            getattr(self, "save_thread", None),
            getattr(self, "compressed_thread", None),
            getattr(self, "statistics_thread", None),
        ]

        for thread in threads:

            if thread is None:
                continue

            if thread is current_thread:
                continue

            if not thread.is_alive():
                continue

            try:
                thread.join(timeout=2.0)

            except Exception:
                pass

        self.get_logger().info(
            "=== RealSense publisher shutdown complete ==="
        )


def main(args=None):

    if not REALSENSE_AVAILABLE:

        print(
            "ERROR: pyrealsense2 not installed"
        )

        return 1

    rclpy.init(args=args)

    # ------------------------------------------------------------------
    # Arguments
    #
    # [serial]
    # --frames-dir PATH
    # --no-save-frames
    # --no-compressed
    # ------------------------------------------------------------------

    serial = None
    frames_dir = None
    no_save_frames = False
    enable_compressed = True

    argv = sys.argv[1:]

    i = 0

    while i < len(argv):

        arg = argv[i]

        if arg == "--frames-dir":

            if i + 1 >= len(argv):

                print(
                    "ERROR: --frames-dir requires path"
                )

                return 1

            frames_dir = argv[i + 1]
            i += 2

        elif arg == "--no-save-frames":

            no_save_frames = True
            i += 1

        elif arg == "--no-compressed":

            enable_compressed = False
            i += 1

        else:

            if serial is not None:

                print(
                    f"ERROR: unknown argument: {arg}"
                )

                return 1

            serial = arg
            i += 1

    publisher = None
    executor = None

    try:

        print(
            "Starting RealSense Publisher..."
        )

        publisher = RealSensePublisher(
            serial=serial,
            frames_dir=frames_dir,
            no_save_frames=no_save_frames,
            enable_compressed=enable_compressed,
        )

        print(
            "Node ready. Press Ctrl+C to exit."
        )

        # --------------------------------------------------------------
        # ROS executor.
        #
        # RealSense capture больше НЕ зависит от executor,
        # потому что работает в своём thread.
        # --------------------------------------------------------------

        executor = (
            rclpy.executors.SingleThreadedExecutor()
        )

        executor.add_node(publisher)

        executor.spin()

    except KeyboardInterrupt:

        print(
            "\nInterrupted by user"
        )

    except Exception as e:

        print(
            f"\nError: {e}"
        )

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


if __name__ == "__main__":
    sys.exit(main())
