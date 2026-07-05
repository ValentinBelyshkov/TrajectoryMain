#!/usr/bin/env python3
"""
MPU-9250/6500 ROS2 Publisher — оптимизированная версия
Блоковое чтение 14 байт за один I2C-транзакцией
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import smbus2
import time
import math
import struct


class MPU6500:
    """Оптимизированный драйвер MPU-6500/9250"""
    
    REG_PWR_MGMT_1 = 0x6B
    REG_ACCEL_CONFIG = 0x1C
    REG_GYRO_CONFIG = 0x1B
    REG_CONFIG = 0x1A
    REG_SMPLRT_DIV = 0x19
    REG_ACCEL_XOUT_H = 0x3B
    REG_WHO_AM_I = 0x75
    
    ACCEL_SCALE_2G = 16384.0
    GYRO_SCALE_250DPS = 131.0
    
    def __init__(self, bus_num=1, address=0x68):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        
        who = self.bus.read_byte_data(self.address, self.REG_WHO_AM_I)
        print(f"[MPU] WHO_AM_I = 0x{who:02X}")
        if who not in (0x68, 0x69, 0x70, 0x71, 0x73):
            raise RuntimeError(f"MPU не найден! WHO_AM_I = 0x{who:02X}")
        
        # Сброс и инициализация
        self.bus.write_byte_data(self.address, self.REG_PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, self.REG_PWR_MGMT_1, 0x01)
        time.sleep(0.05)
        self.bus.write_byte_data(self.address, self.REG_CONFIG, 0x01)  # DLPF 184 Hz
        self.bus.write_byte_data(self.address, self.REG_SMPLRT_DIV, 0x04)  # 200 Hz
        self.bus.write_byte_data(self.address, self.REG_ACCEL_CONFIG, 0x00)  # +/- 2g
        self.bus.write_byte_data(self.address, self.REG_GYRO_CONFIG, 0x00)   # +/- 250 dps
        time.sleep(0.05)
        
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.accel_bias = [0.0, 0.0, 0.0]
        
    def _read_block(self, reg, length):
        """Блоковое чтение через i2c_rdwr — один ioctl вместо многих"""
        write = smbus2.i2c_msg.write(self.address, [reg])
        read = smbus2.i2c_msg.read(self.address, length)
        self.bus.i2c_rdwr(write, read)
        return list(read)
    
    def read_all(self):
        """Чтение акселерометра, температуры и гироскопа за 1 транзакцию"""
        data = self._read_block(self.REG_ACCEL_XOUT_H, 14)
        
        def to_int16(h, l):
            v = (h << 8) | l
            return v if v < 32768 else v - 65536
        
        ax = to_int16(data[0], data[1])
        ay = to_int16(data[2], data[3])
        az = to_int16(data[4], data[5])
        # temp = to_int16(data[6], data[7])
        gx = to_int16(data[8], data[9])
        gy = to_int16(data[10], data[11])
        gz = to_int16(data[12], data[13])
        
        d2r = math.pi / 180.0
        return [
            (ax / self.ACCEL_SCALE_2G) * 9.80665 - self.accel_bias[0],
            (ay / self.ACCEL_SCALE_2G) * 9.80665 - self.accel_bias[1],
            (az / self.ACCEL_SCALE_2G) * 9.80665 - self.accel_bias[2],
            (gx / self.GYRO_SCALE_250DPS) * d2r - self.gyro_bias[0],
            (gy / self.GYRO_SCALE_250DPS) * d2r - self.gyro_bias[1],
            (gz / self.GYRO_SCALE_250DPS) * d2r - self.gyro_bias[2],
        ]
    
    def calibrate(self, samples=400):
        print("[MPU] Калибровка, не двигайте...")
        sg = [0.0, 0.0, 0.0]
        sa = [0.0, 0.0, 0.0]
        for _ in range(samples):
            d = self.read_all()
            sa[0] += d[0]; sa[1] += d[1]; sa[2] += d[2]
            sg[0] += d[3]; sg[1] += d[4]; sg[2] += d[5]
            time.sleep(0.005)
        
        self.accel_bias = [sa[i]/samples for i in range(3)]
        self.accel_bias[2] -= 9.80665  # Z должен быть 1g
        self.gyro_bias = [sg[i]/samples for i in range(3)]
        print(f"[MPU] Accel bias: [{self.accel_bias[0]:.4f}, {self.accel_bias[1]:.4f}, {self.accel_bias[2]:.4f}]")
        print(f"[MPU] Gyro  bias: [{self.gyro_bias[0]:.4f}, {self.gyro_bias[1]:.4f}, {self.gyro_bias[2]:.4f}]")
    
    def close(self):
        self.bus.close()


class MPU6500Publisher(Node):
    def __init__(self):
        super().__init__('mpu6500_publisher')
        
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 200.0)
        
        bus = self.get_parameter('i2c_bus').value
        addr = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate').value
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200
        )
        
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos)
        self.mpu = MPU6500(bus, addr)
        self.mpu.calibrate(400)
        
        period = 1.0 / rate
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(f'Публикация IMU с частотой {rate} Hz')
        
        self.frame_count = 0
        self.start_time = self.get_clock().now()
        
    def timer_callback(self):
        try:
            d = self.mpu.read_all()
            stamp = self.get_clock().now().to_msg()
            
            msg = Imu()
            msg.header = Header(stamp=stamp, frame_id=self.frame_id)
            msg.linear_acceleration.x = float(d[0])
            msg.linear_acceleration.y = float(d[1])
            msg.linear_acceleration.z = float(d[2])
            msg.angular_velocity.x = float(d[3])
            msg.angular_velocity.y = float(d[4])
            msg.angular_velocity.z = float(d[5])
            
            # ВСЕ значения должны быть float (0.0, не 0)
            msg.linear_acceleration_covariance = [
                0.0004, 0.0, 0.0,
                0.0, 0.0004, 0.0,
                0.0, 0.0, 0.0004
            ]
            msg.angular_velocity_covariance = [
                0.000064, 0.0, 0.0,
                0.0, 0.000064, 0.0,
                0.0, 0.0, 0.000064
            ]
            # Полный массив, не через индекс
            msg.orientation_covariance = [
                -1.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0
            ]
            
            self.imu_pub.publish(msg)
            self.frame_count += 1
            
            if self.frame_count % 1000 == 0:
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.get_logger().info(f'IMU: {self.frame_count} msgs, rate={self.frame_count/elapsed:.1f} Hz')
                
        except Exception as e:
            self.get_logger().error(f'Ошибка: {e}')
            
    def destroy_node(self):
        self.mpu.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MPU6500Publisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
