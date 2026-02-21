import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import math
import numpy as np

class FlightLogPublisher(Node):
    def __init__(self):
        super().__init__('flight_log_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_flight_log', 10)
        self.timer = self.create_timer(1.0, self.publish_logs)
        self.logs = self.parse_logs()
        self.index = 0

        # GCPs сгруппированы по изображениям (UTM easting, northing, height=0)
        self.gcps = {
            '000001': np.array([
                [476900.02, 6149546.86, 0],
                [476902.35, 6149554.46, 0],
                [476896.00, 6149556.61, 0]
            ]),
            '000091': np.array([
                [476855.52, 6149593.18, 0],
                [476901.47, 6149564.70, 0],
                [476915.26, 6149576.80, 0]
            ]),
            '000205': np.array([
                [476882.75, 6149647.32, 0],
                [476897.65, 6149676.83, 0],
                [476870.07, 6149619.48, 0]
            ])
        }

        # Средние UTM позиции для каждого фото (как proxy для позиции камеры, но добавьте реальную высоту если известна)
        global_means = {
            '000001': np.mean(self.gcps['000001'], axis=0),
            '000091': np.mean(self.gcps['000091'], axis=0),
            '000205': np.mean(self.gcps['000205'], axis=0)
        }

        # Предполагаемые corresponding local poses из логов (адаптируйте индексы!)
        local_points = np.array([
            [self.logs[0]['x'], self.logs[0]['y'], self.logs[0]['z']],  # для 000001
            [self.logs[4]['x'], self.logs[4]['y'], self.logs[4]['z']],  # для 000091 (пример)
            [self.logs[8]['x'], self.logs[8]['y'], self.logs[8]['z']]   # для 000205
        ])
        global_points = np.array([
            global_means['000001'],
            global_means['000091'],
            global_means['000205']
        ])

        # Вычисление матрицы трансформации (affine 4x4)
        self.transformation_matrix = self.compute_transformation(local_points, global_points)
        self.get_logger().info(f'Матрица трансформа: \n{self.transformation_matrix}')

        # Для конвертации UTM в lat/lon после трансформа
        self.zone = 37
        self.northern = True

    def compute_transformation(self, local_points, global_points):
        # Affine transformation: global = A @ local + b, но в матрице 4x4
        # Добавляем столбец 1 для homogeneous
        n = local_points.shape[0]
        if n < 3:
            raise ValueError("Нужны минимум 3 пары точек")
        
        Local = np.hstack((local_points, np.ones((n, 1))))
        Global = global_points

        # Least squares: A = Global @ pinv(Local), но по столбцам
        A = np.zeros((3, 4))
        for i in range(3):
            A[i] = np.linalg.lstsq(Local, Global[:, i], rcond=None)[0]
        
        # Полная 4x4 матрица
        T = np.eye(4)
        T[:3, :] = A
        return T

    def utm_to_latlon(self, easting, northing, height, zone, northern=True):
        # Та же функция как раньше (копирую для полноты)
        a = 6378137.0
        f = 1 / 298.257223563
        b = a * (1 - f)
        e2 = 1 - (b**2 / a**2)
        e = math.sqrt(e2)
        e1 = (1 - math.sqrt(1 - e2)) / (1 + math.sqrt(1 - e2))
        k0 = 0.9996

        lon0 = 6 * zone - 183

        x = easting - 500000
        y = northing if northern else northing - 10000000

        M = y / k0
        mu = M / (a * (1 - e2/4 - 3*e2**2/64 - 5*e2**3/256))

        phi1 = mu + (3*e1/2 - 27*e1**3/32)*math.sin(2*mu) + (21*e1**2/16 - 55*e1**4/32)*math.sin(4*mu) + (151*e1**3/96)*math.sin(6*mu) + (1097*e1**4/512)*math.sin(8*mu)

        e_prime2 = e2 / (1 - e2)
        C1 = e_prime2 * math.cos(phi1)**2
        T1 = math.tan(phi1)**2
        N1 = a / math.sqrt(1 - e2 * math.sin(phi1)**2)
        R1 = a * (1 - e2) / (1 - e2 * math.sin(phi1)**2)**1.5
        D = x / (N1 * k0)

        lat_rad = phi1 - (N1 * math.tan(phi1) / R1) * (D**2/2 - (5 + 3*T1 + 10*C1 - 4*C1**2 - 9*e_prime2)*D**4/24 + (61 + 90*T1 + 298*C1 + 45*T1**2 - 252*e_prime2 - 3*C1**2)*D**6/720)
        lat_deg = math.degrees(lat_rad)

        delta_lon_rad = (D - (1 + 2*T1 + C1)*D**3/6 + (5 - 2*C1 + 28*T1 - 3*C1**2 + 8*e_prime2 + 24*T1**2)*D**5/120) / math.cos(phi1)
        lon_deg = lon0 + math.degrees(delta_lon_rad)

        return lat_deg, lon_deg, height  # height без изменений

    def parse_logs(self):
        # Ваши логи (x, y, z)
        logs = [
            {'x': -4.858016490936279, 'y': -4.307590007781982, 'z': 2.1660401821136475},
            # ... (остальные, как раньше)
            {'x': -4.857174396514893, 'y': -4.013225555419922, 'z': 2.2129011154174805},
        ]
        return logs

    def publish_logs(self):
        if self.index >= len(self.logs):
            self.get_logger().info('Все логи опубликованы.')
            self.timer.cancel()
            return

        log = self.logs[self.index]
        local_hom = np.array([log['x'], log['y'], log['z'], 1.0])

        # Применяем трансформацию: получаем UTM
        global_utm = self.transformation_matrix @ local_hom
        easting, northing, height = global_utm[:3]

        # Конвертируем UTM в lat/lon
        lat, lon, alt = self.utm_to_latlon(easting, northing, height, self.zone, self.northern)

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.publisher_.publish(msg)
        self.get_logger().info(f'Опубликовано GPS: lat={lat:.8f}, lon={lon:.8f}, alt={alt:.2f}')
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = FlightLogPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
