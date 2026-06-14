#!/usr/bin/env python3
"""
ROS 2 GPS Bridge Node
Listen to /camera/gps, encode via selected protocol, send via UART/I2C/USB.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import queue
import threading
import time
import struct

# Protocol encoders
from mavlink_encoder import MavlinkEncoder
from msp_encoder import MspEncoder
from ublox_encoder import UbloxEncoder


class HardwareType:
    UART = "uart"
    I2C = "i2c"
    USB = "usb"


class GpsBridgeNode(Node):
    def __init__(self):
        super().__init__('gps_bridge')

        # ── Parameters ──
        self.declare_parameter('protocol', 'mavlink')
        self.declare_parameter('hw_type', 'uart')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x42)

        self.protocol = self.get_parameter('protocol').value
        self.hw_type = self.get_parameter('hw_type').value
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_addr = self.get_parameter('i2c_addr').value

        # ── Encoders ──
        self.encoder = self._init_encoder()

        # ── ROS Subscription ──
        self.gps_sub = self.create_subscription(
            NavSatFix, '/camera/gps', self.on_gps,
            rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        )

        # ── Communication Layer ──
        self.send_queue = queue.Queue(maxsize=50)
        self.hw_interface = None
        self.running = True

        self.comm_thread = threading.Thread(target=self._comm_loop, daemon=True)
        self.comm_thread.start()

        self._init_hw()
        self.get_logger().info(f'✅ GPS Bridge started: {self.protocol} over {self.hw_type}')

    def _init_encoder(self):
        if self.protocol == 'mavlink':
            return MavlinkEncoder()
        if self.protocol == 'msp':
            return MspEncoder()
        if self.protocol == 'ublox':
            return UbloxEncoder()
        raise ValueError(f"Unsupported protocol: {self.protocol}")

    def _init_hw(self):
        try:
            if self.hw_type in (HardwareType.UART, HardwareType.USB):
                import serial
                self.hw_interface = serial.Serial(
                    port=self.port, baudrate=self.baudrate,
                    timeout=0.1, write_timeout=0.5
                )
                self.get_logger().info(f'🔌 UART/USB opened: {self.port} @ {self.baudrate}')
            elif self.hw_type == HardwareType.I2C:
                import smbus2
                self.hw_interface = smbus2.SMBus(self.i2c_bus)
                self.get_logger().info(f'🔌 I2C opened: Bus {self.i2c_bus}, Addr 0x{self.i2c_addr:02X}')
            else:
                raise ValueError(f"Unsupported hw_type: {self.hw_type}")
        except Exception as e:
            self.get_logger().error(f'❌ Failed to init hardware: {e}')
            self.hw_interface = None

    def on_gps(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        data = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'fix': msg.status.status,
            'ts': self.get_clock().now().nanoseconds // 1000
        }
        try:
            packet = self.encoder.encode(data)
            if not self.send_queue.full():
                self.send_queue.put(packet, block=False)
        except Exception as e:
            self.get_logger().error(f'Encoding failed: {e}')

    def _comm_loop(self):
        while self.running:
            try:
                if self.hw_interface is None:
                    time.sleep(1.0)
                    self._init_hw()
                    continue

                if self.send_queue.empty():
                    time.sleep(0.01)
                    continue

                packet = self.send_queue.get_nowait()

                if self.hw_type == HardwareType.I2C:
                    chunk_size = 28
                    for i in range(0, len(packet), chunk_size):
                        self.hw_interface.write_i2c_block_data(
                            self.i2c_addr, 0x00, list(packet[i:i+chunk_size])
                        )
                else:
                    self.hw_interface.write(packet)

            except Exception as e:
                self.get_logger().error(f'💥 Comm error: {e}')
                if self.hw_interface and hasattr(self.hw_interface, 'close'):
                    try:
                        self.hw_interface.close()
                    except:
                        pass
                self.hw_interface = None
                time.sleep(2.0)

    def destroy_node(self):
        self.running = False
        if self.hw_interface and hasattr(self.hw_interface, 'close'):
            try:
                self.hw_interface.close()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
    # ROS 2 сам закроет контекст — не вызываем shutdown() явно


if __name__ == '__main__':
    main()
