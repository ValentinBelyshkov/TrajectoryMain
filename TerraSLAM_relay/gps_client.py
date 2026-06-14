#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix, NavSatStatus
import numpy as np
import sys


class Relay(Node):
    def __init__(self, path):
        super().__init__('relay')
        self.pub = self.create_publisher(NavSatFix, '/camera/gps', 10)
        self.create_subscription(Pose, '/camera_pose', self.cb, 10)

        pts = []
        with open(path) as f:
            for l in f:
                l = l.strip()
                if not l or l[0] == '#':
                    continue
                a, b = l.split(';')
                lat, lon = map(float, a.split())
                x, y, z = map(float, b.split())
                pts.append((lat, lon, x, y, z))

        if not pts:
            raise ValueError(f"No valid points loaded from {path}")

        # === ADD THESE DEBUG CHECKS ===
        print(f"Loaded {len(pts)} points")
        X = np.array([[z, y, 1.0] for _, _, _, y, z in pts])
        y_lat = np.array([p[0] for p in pts])
        y_lon = np.array([p[1] for p in pts])

        print("X shape:", X.shape)
        print("Any NaN in X?", np.any(np.isnan(X)))
        print("Any Inf in X?", np.any(np.isinf(X)))
        print("Any NaN in lat?", np.any(np.isnan(y_lat)))
        print("Any NaN in lon?", np.any(np.isnan(y_lon)))

        # Optional: check for constant columns (rank deficiency)
        print("X std per column:", X.std(axis=0))
        # ==============================

        self.a, *_ = np.linalg.lstsq(X, y_lat, rcond=None)
        self.b, *_ = np.linalg.lstsq(X, y_lon, rcond=None)
        self.get_logger().info(f'Loaded {len(pts)} GCPs')

    def cb(self, msg: Pose):
        x, y, z = msg.position.x, msg.position.y, msg.position.z

        # ORB-SLAM3 спец-значения при потере трекинга
        if abs(x + 3.0) < 0.01 and abs(y + 3.0) < 0.01 and abs(z + 3.0) < 0.01:
            return

        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = 'camera'
        gps.status.status = NavSatStatus.STATUS_FIX
        gps.latitude = float(self.a[0] * z + self.a[1] * y + self.a[2])
        gps.longitude = float(self.b[0] * z + self.b[1] * y + self.b[2])
        gps.altitude = float(x)
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub.publish(gps)
        self.get_logger().info(f'{gps.latitude:.7f} {gps.longitude:.7f} {gps.altitude:.1f}')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Relay(sys.argv[1] if len(sys.argv) > 1 else 'calib.txt'))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
