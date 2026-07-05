#!/usr/bin/env python3
import json
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Int8

# Файл, который system_manager читает через GET /api/v1/slam/status
STATUS_FILE = "/tmp/terraslam_slam_status"

# Соответствие int-кода ORB-SLAM3 строковому имени состояния
TRACKING_STATE_NAMES = {
    -1: "UNKNOWN",
    0:  "NO_IMAGES_YET",
    1:  "NOT_INITIALIZED",
    2:  "OK",
    3:  "TRACKING_LOST",
}

class GcpRelay(Node):
    def __init__(self, path):
        super().__init__('gcp_relay')
        self.pub = self.create_publisher(NavSatFix, '/camera/gps', 10)
        self.create_subscription(Pose, '/camera_pose', self.on_pose, 10)
        self.create_subscription(Int8, '/orb_slam3/tracking_state', self._cb_tracking, 10)

        # Внутреннее состояние для записи статуса
        self._tracking_state: int = -1   # из /orb_slam3/tracking_state
        self._pose_state: int = -1       # выведенное из Pose-значений
        self._last_pose_ts: float = 0.0  # время последнего сообщения с позой

        # Таймер: писать статус раз в секунду
        self.create_timer(1.0, self._write_status)

        pts = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line[0] in ('#', '+'):
                    continue
                vals = line.split()
                if len(vals) < 5:
                    continue
                # Формат файла точно как у тебя:  x y z lon lat alt
                px, py, pz = float(vals[0]), float(vals[1]), float(vals[2])
                lon, lat = float(vals[3]), float(vals[4])
                alt = float(vals[5]) if len(vals) >= 6 else 0.0
                pts.append((px, py, lat, lon, alt))

        if len(pts) < 3:
            raise ValueError(f"Нужно минимум 3 GCP точки, загружено {len(pts)}")

        self.get_logger().info(f"✅ Загружено {len(pts)} точек калибровки")

        # Z полностью исключаем из расчета, он только портит результат
        X = np.array([[px, py, 1.0] for px, py, _, _, _ in pts])
        y_lat = np.array([p[2] for p in pts])
        y_lon = np.array([p[3] for p in pts])

        # Считаем оптимальное преобразование
        self.a, residuals_lat, _, _ = np.linalg.lstsq(X, y_lat, rcond=None)
        self.b, residuals_lon, _, _ = np.linalg.lstsq(X, y_lon, rcond=None)

        # Самая полезная строчка для отладки
        rmse_lat = np.sqrt(residuals_lat[0]/len(pts)) * 111111
        rmse_lon = np.sqrt(residuals_lon[0]/len(pts)) * 111111 * np.cos(np.deg2rad(55.5))

        self.get_logger().info(f"Средняя ошибка преобразования:")
        self.get_logger().info(f"  Широта: {rmse_lat:.2f} м")
        self.get_logger().info(f"  Долгота: {rmse_lon:.2f} м")

        if rmse_lat > 0.5 or rmse_lon > 0.5:
            self.get_logger().warn("⚠️ Большая ошибка! Проверь что ты не перепутал порядок координат в GPC файле")

    def _cb_tracking(self, msg: Int8):
        """Прямой Int8 из /orb_slam3/tracking_state — приоритетный источник."""
        self._tracking_state = int(msg.data)

    def on_pose(self, msg: Pose):
        x, y, z = msg.position.x, msg.position.y, msg.position.z
        self._last_pose_ts = time.time()

        # Декодируем специальные значения позы → состояние трекинга
        if abs(x + 3.0) < 0.01 and abs(y + 3.0) < 0.01 and abs(z + 3.0) < 0.01:
            self._pose_state = 3   # TRACKING_LOST
            return
        if abs(x + 1.0) < 0.01 and abs(y + 1.0) < 0.01 and abs(z + 1.0) < 0.01:
            self._pose_state = 1   # NOT_INITIALIZED
            return
        if abs(x) < 0.01 and abs(y) < 0.01 and abs(z) < 0.01:
            self._pose_state = 0   # NO_IMAGES_YET
            return

        self._pose_state = 2   # OK

        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = 'map'
        gps.status.status = NavSatStatus.STATUS_FIX

        gps.latitude  = self.a[0] * x + self.a[1] * y + self.a[2]
        gps.longitude = self.b[0] * x + self.b[1] * y + self.b[2]
        gps.altitude  = z

        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.pub.publish(gps)
        self.get_logger().info(
            f'ORB: {x:+.2f} {y:+.2f} {z:+.2f} | GPS: {gps.latitude:.7f} {gps.longitude:.7f}',
            throttle_duration_sec=0.2
        )

    def _write_status(self):
        """Раз в секунду пишет состояние SLAM в /tmp/terraslam_slam_status.
        Этот файл читает system_manager через GET /api/v1/slam/status."""
        # Приоритет: Int8-топик, иначе — из Pose-значений
        state = self._tracking_state if self._tracking_state != -1 else self._pose_state
        state_name = TRACKING_STATE_NAMES.get(state, "UNKNOWN")
        initialized = (state == 2)

        # Если давно не приходила поза — помечаем NO_IMAGES
        if self._last_pose_ts > 0 and (time.time() - self._last_pose_ts) > 5.0:
            state = 0
            state_name = "NO_IMAGES_YET"
            initialized = False

        status = {
            "slam_state":      state,
            "slam_state_name": state_name,
            "initialized":     initialized,
            "drone_mode":      "LOCALIZATION_ONLY",
            "timestamp":       time.time(),
        }

        try:
            with open(STATUS_FILE, "w") as f:
                json.dump(status, f)
        except Exception as e:
            self.get_logger().warn(f"Failed to write status file: {e}")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GcpRelay(sys.argv[1] if len(sys.argv) > 1 else 'calib.gpc'))
    rclpy.shutdown()


if __name__ == '__main__':
    main()

