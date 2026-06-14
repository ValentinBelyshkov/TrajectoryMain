#!/usr/bin/env python3
"""geo_matcher_node.py — ищет кадр на карте, выдаёт Sim3"""
import rospy
import numpy as np
import cv2
import pickle
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

class GeoMatcher:
    def __init__(self):
        rospy.init_node('geo_matcher')
        
        # --- Параметры ---
        self.map_path = rospy.get_param('~map_path', 'map_data.pkl')
        self.altitude = rospy.get_param('~altitude', 1000.0)  # метры
        self.focal_mm = rospy.get_param('~focal_length_mm', 24.0)
        self.sensor_mm = rospy.get_param('~sensor_width_mm', 13.2)
        self.img_w = rospy.get_param('~image_width_px', 1920)
        self.img_h = rospy.get_param('~image_height_px', 1080)
        
        # GSD камеры (м/пиксель)
        self.gsd_cam = (self.altitude * self.sensor_mm) / (self.focal_mm * self.img_w)
        rospy.loginfo(f"Camera GSD: {self.gsd_cam:.3f} m/px")
        
        # --- Загрузка карты ---
        rospy.loginfo("Loading map...")
        with open(self.map_path, 'rb') as f:
            self.map_data = pickle.load(f)
        
        self.gray_map = self.map_data['gray']
        self.kp_map = self.map_data['kp']
        self.des_map = self.map_data['des']
        self.transform = self.map_data['transform']  # rasterio.Affine
        self.map_h, self.map_w = self.gray_map.shape
        
        self.gsd_map = self.transform.a  # разрешение по X (м/пиксель)
        rospy.loginfo(f"Map loaded: {self.map_w}x{self.map_h}, GSD={self.gsd_map:.2f} m/px")
        rospy.loginfo(f"Map ORB features: {len(self.kp_map)}")
        
        # --- ROS ---
        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber('/camera/image_raw', Image, self.img_cb)
        self.pub_transform = rospy.Publisher('/geo_transform', PoseStamped, queue_size=1)
        self.tf_br = TransformBroadcaster()
        
        self.last_img = None
        self.initialized = False
        
        # Таймер: матчим раз в секунду
        self.timer = rospy.Timer(rospy.Duration(1.0), self.match_cb)
        rospy.loginfo("GeoMatcher ready. Waiting for images...")

    def img_cb(self, msg):
        try:
            self.last_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(f"CV bridge: {e}")

    def pixel_to_latlon(self, px, py):
        """Пиксели карты → lat/lon (для EPSG:4326)"""
        # rasterio transform: (col, row) → (x, y)
        # Для WGS84: x=lon, y=lat
        lon, lat = self.transform * (px, py)
        return lat, lon

    def match_cb(self, event):
        if self.last_img is None:
            return
        
        gray_drone = cv2.cvtColor(self.last_img, cv2.COLOR_BGR2GRAY)
        
        # CLAHE для компенсации разного освещения
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray_drone = clahe.apply(gray_drone)
        
        # --- 1. Быстрый грубый поиск: Phase Correlation ---
        # Масштабируем кадр под GSD карты
        scale_factor = self.gsd_cam / self.gsd_map
        drone_resized = cv2.resize(gray_drone, 
                                   (int(self.img_w * scale_factor), 
                                    int(self.img_h * scale_factor)))
        
        if drone_resized.shape[0] > self.map_h or drone_resized.shape[1] > self.map_w:
            rospy.logwarn_throttle(5.0, "Drone image larger than map!")
            return
        
        # Phase correlation для грубого смещения
        # Работает если изображения похожи (nadir camera, похожее освещение)
        try:
            (sx, sy), response = cv2.phaseCorrelate(np.float32(self.gray_map), 
                                                     np.float32(drone_resized))
        except:
            rospy.logwarn("Phase correlation failed")
            return
        
        # Округляем до целых
        cx = int(round(sx))
        cy = int(round(sy))
        
        # Проверяем границы
        dh, dw = drone_resized.shape
        if cx < 0 or cy < 0 or cx + dw > self.map_w or cy + dh > self.map_h:
            rospy.logwarn_throttle(5.0, f"Phase correlation out of bounds: ({cx}, {cy})")
            return
        
        rospy.loginfo(f"Phase corr: ({cx}, {cy}), response={response:.3f}")
        
        # --- 2. Уточнение: ORB + RANSAC на окрестности ---
        # Вырезаем ROI из карты
        roi_map = self.gray_map[cy:cy+dh, cx:cx+dw]
        
        # ORB на кадре дрона
        orb = cv2.ORB_create(nfeatures=2000, scaleFactor=1.2, nlevels=8)
        kp_drone, des_drone = orb.detectAndCompute(drone_resized, None)
        if des_drone is None or len(kp_drone) < 20:
            rospy.logwarn("Not enough features on drone image")
            return
        
        # ORB на ROI карты
        kp_roi, des_roi = orb.detectAndCompute(roi_map, None)
        if des_roi is None or len(kp_roi) < 20:
            rospy.logwarn("Not enough features on map ROI")
            return
        
        # Matching
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        matches = bf.knnMatch(des_drone, des_roi, k=2)
        
        good = []
        for m_n in matches:
            if len(m_n) == 2:
                m, n = m_n
                if m.distance < 0.75 * n.distance:
                    good.append(m)
        
        if len(good) < 10:
            rospy.logwarn(f"Too few good matches: {len(good)}")
            return
        
        src = np.float32([kp_drone[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst = np.float32([kp_roi[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        
        # RANSAC Affine (Similarity: scale + rotation + translation)
        M, mask = cv2.estimateAffinePartial2D(src, dst, 
                                               method=cv2.RANSAC,
                                               ransacReprojThreshold=3.0,
                                               maxIters=2000,
                                               confidence=0.99)
        if M is None:
            rospy.logwarn("RANSAC failed")
            return
        
        inliers = int(np.sum(mask)) if mask is not None else 0
        if inliers < 6:
            rospy.logwarn(f"Too few inliers: {inliers}")
            return
        
        # --- 3. Извлекаем параметры трансформации ---
        # M = [ [scale*cos(yaw), -scale*sin(yaw), tx],
        #       [scale*sin(yaw),  scale*cos(yaw), ty] ]
        a, b, tx = M[0]
        c, d, ty = M[1]
        
        scale = math.sqrt(a*a + b*b)
        yaw = math.atan2(c, a)  # или atan2(b, a) в зависимости от знака
        
        rospy.loginfo(f"Match OK: scale={scale:.3f}, yaw={math.degrees(yaw):.1f}°, "
                      f"inliers={inliers}/{len(good)}")
        
        # --- 4. Центр кадра на карте ---
        # Центр resized кадра
        cx_drone = dw / 2.0
        cy_drone = dh / 2.0
        
        # Применяем affine transform
        cx_map = a * cx_drone + b * cy_drone + tx + cx
        cy_map = c * cx_drone + d * cy_drone + ty + cy
        
        # Переводим в lat/lon
        lat, lon = self.pixel_to_latlon(cx_map, cy_map)
        
        # --- 5. Публикуем результат ---
        now = rospy.Time.now()
        
        # PoseStamped как "transform" (не совсем Pose, но удобно)
        # position = lat, lon, alt
        # orientation = yaw
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x = lon
        pose.pose.position.y = lat
        pose.pose.position.z = self.altitude
        
        # Кватернион из yaw (roll=pitch=0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.pub_transform.publish(pose)
        
        # TF для визуализации
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "geo_matcher"
        t.transform.translation.x = lon
        t.transform.translation.y = lat
        t.transform.translation.z = self.altitude
        t.transform.rotation = pose.pose.orientation
        self.tf_br.sendTransform(t)
        
        # Логируем JSON-подобный вывод
        rospy.loginfo(
            f"\n=== GEO TRANSFORM ===\n"
            f"scale: {scale * self.gsd_map / self.gsd_cam:.6f}\n"
            f"rotation (yaw): {math.degrees(yaw):.3f}\n"
            f"translation: [{lon:.6f}, {lat:.6f}, {self.altitude:.1f}]\n"
            f"lat0: {lat:.9f}\n"
            f"lon0: {lon:.9f}\n"
            f"alt0: {self.altitude:.1f}\n"
            f"====================="
        )
        
        self.initialized = True

if __name__ == '__main__':
    try:
        GeoMatcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
