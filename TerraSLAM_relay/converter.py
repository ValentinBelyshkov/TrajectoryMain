#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from geographiclib.geodesic import Geodesic
import numpy as np
import sys
import os

class PoseToGPSConverter(Node):
    def __init__(self):
        super().__init__('pose_to_gps')

        # Получаем параметр calib_file
        self.declare_parameter('calib_file', 'calib.txt')
        calib_file = self.get_parameter('calib_file').value

        if not os.path.exists(calib_file):
            self.get_logger().fatal(f'Файл калибровки не найден: {calib_file}')
            rclpy.shutdown()
            sys.exit(1)

        self.geod = Geodesic.WGS84
        self.load_calibration(calib_file)
        self.build_transform()

        self.subscription = self.create_subscription(
            Pose,
            '/camera_pose',
            self.pose_callback,
            10
        )
        self.publisher = self.create_publisher(PoseStamped, '/gps_pose', 10)

        self.get_logger().info('PoseToGPSConverter запущен. Ожидаю /camera_pose...')

    def load_calibration(self, calib_file):
        self.local_points = []
        self.gps_points = []

        with open(calib_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                try:
                    local_part, gps_part = line.split(';')
                    x, y, z = map(float, local_part.split())
                    lat, lon = map(float, gps_part.split())
                    self.local_points.append([x, y])
                    self.gps_points.append([lat, lon])
                except Exception as e:
                    self.get_logger().warn(f'Ошибка парсинга строки: {line} — {e}')

        if len(self.local_points) < 3:
            self.get_logger().fatal('Нужно минимум 3 калибровочные точки!')
            rclpy.shutdown()
            sys.exit(1)

        self.get_logger().info(f'Загружено {len(self.local_points)} калибровочных точек')

    def build_transform(self):
        lat0, lon0 = self.gps_points[0]
        enu_points = []

        for lat, lon in self.gps_points:
            inv = self.geod.Inverse(lat0, lon0, lat, lon)
            s12 = inv['s12']
            azi1 = np.radians(inv['azi1'])
            north = s12 * np.cos(azi1)
            east  = s12 * np.sin(azi1)
            enu_points.append([east, north])

        self.origin_lat, self.origin_lon = lat0, lon0
        self.enu_points = np.array(enu_points)
        self.local_points_arr = np.array(self.local_points)

        N = len(self.local_points)
        X = np.hstack([self.local_points_arr, np.ones((N, 1))])  # (N, 3)
        Y = self.enu_points  # (N, 2)

        # Аффинное преобразование: [x, y, 1] @ T = [east, north]
        self.T = np.linalg.lstsq(X, Y, rcond=None)[0]  # shape (3, 2)

        self.get_logger().info('Аффинное преобразование построено')

    def local_to_enu(self, x, y):
        vec = np.array([x, y, 1.0])
        east, north = vec @ self.T
        return east, north

    def enu_to_gps(self, east, north):
        distance = np.hypot(east, north)
        if distance < 1e-6:
            return self.origin_lat, self.origin_lon
        azimuth = np.degrees(np.arctan2(east, north))  # в градусах, от севера
        direct = self.geod.Direct(self.origin_lat, self.origin_lon, azimuth, distance)
        return direct['lat2'], direct['lon2']

    def pose_callback(self, msg):
        x = msg.position.x
        y = msg.position.y

        try:
            east, north = self.local_to_enu(x, y)
            lat, lon = self.enu_to_gps(east, north)

            # Создаём PoseStamped для публикации (с header)
            out_msg = PoseStamped()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = "map"  # или как вам нужно
            out_msg.pose.position.x = lat
            out_msg.pose.position.y = lon
            out_msg.pose.position.z = 0.0
            out_msg.pose.orientation = msg.orientation  # копируем ориентацию

            self.publisher.publish(out_msg)
            self.get_logger().info(f'GPS: {lat:.8f}, {lon:.8f}')
        except Exception as e:
            self.get_logger().error(f'Ошибка преобразования: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseToGPSConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
