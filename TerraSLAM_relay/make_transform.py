import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import json

def compute_similarity_transform(gps_points, orb_points, lat0, lon0, alt0):
    """
    Вычисляет Similarity Transform (scale, rotation, translation)
    из локальных координат ORB-SLAM3 в глобальные ECEF/ENU.
    """
    # Преобразуем GPS (lat, lon, alt) в локальную ENU систему
    def geodetic_to_enu(points, lat0, lon0, alt0):
        # WGS-84 параметры
        a = 6378137.0  # полуось
        f = 1 / 298.257223563
        e_sq = 2 * f - f ** 2
        
        def geodetic_to_ecef(lat, lon, alt):
            lat_rad = np.radians(lat)
            lon_rad = np.radians(lon)
            N = a / np.sqrt(1 - e_sq * np.sin(lat_rad) ** 2)
            x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
            y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
            z = (N * (1 - e_sq) + alt) * np.sin(lat_rad)
            return np.array([x, y, z])
        
        ref_ecef = geodetic_to_ecef(lat0, lon0, alt0)
        
        enu_points = []
        for lat, lon, alt in points:
            ecef = geodetic_to_ecef(lat, lon, alt)
            dx = ecef - ref_ecef
            
            lat_rad = np.radians(lat0)
            lon_rad = np.radians(lon0)
            
            # Матрица поворота ECEF -> ENU
            t = np.array([
                [-np.sin(lon_rad), np.cos(lon_rad), 0],
                [-np.sin(lat_rad) * np.cos(lon_rad), -np.sin(lat_rad) * np.sin(lon_rad), np.cos(lat_rad)],
                [np.cos(lat_rad) * np.cos(lon_rad), np.cos(lat_rad) * np.sin(lon_rad), np.sin(lat_rad)]
            ])
            
            enu = t @ dx
            enu_points.append(enu)
        
        return np.array(enu_points)
    
    # Конвертируем GPS в ENU (метры)
    enu_points = geodetic_to_enu(gps_points, lat0, lon0, alt0)
    
    # ORB точки (убираем транспонирование, работаем с (N, 3))
    orb = orb_points.T if orb_points.shape[0] == 3 else orb_points
    
    # Центроиды
    centroid_orb = np.mean(orb, axis=0)
    centroid_enu = np.mean(enu_points, axis=0)
    
    # Центрируем
    orb_centered = orb - centroid_orb
    enu_centered = enu_points - centroid_enu
    
    # Вычисляем масштаб
    scale = np.sqrt(np.sum(enu_centered ** 2)) / np.sqrt(np.sum(orb_centered ** 2))
    
    # Вычисляем поворот (Kabsch algorithm)
    H = orb_centered.T @ enu_centered
    U, S, Vt = np.linalg.svd(H)
    R_matrix = Vt.T @ U.T
    
    # Проверка на reflection
    if np.linalg.det(R_matrix) < 0:
        Vt[-1, :] *= -1
        R_matrix = Vt.T @ U.T
    
    # Вычисляем трансляцию
    translation = centroid_enu - scale * R_matrix @ centroid_orb
    
    return {
        "scale": float(scale),
        "rotation": R_matrix.tolist(),
        "translation": translation.tolist(),
        "lat0": float(lat0),
        "lon0": float(lon0),
        "alt0": float(alt0)
    }


# ==================== НАБОР 1: Pittsburgh ====================
print("=" * 60)
print("НАБОР 1: Pittsburgh")
print("=" * 60)

gps_points_1 = np.array([
    [40.413552, -79.948717, 200.5],  # точка 1
    [40.413600, -79.948800, 201.0],  # точка 2
    [40.413500, -79.948600, 199.8],  # точка 3
    [40.413650, -79.948550, 200.2],  # точка 4
    [40.413480, -79.948750, 200.9],  # точка 5
])

# 3D-координаты в ORB-SLAM3 (произвольные единицы)
orb_points_1 = np.array([
    [0.5, 1.2, 3.1],   # точка 1
    [2.1, 0.8, 2.9],   # точка 2
    [-0.3, 1.5, 3.3],  # точка 3
    [1.8, -0.2, 2.7],  # точка 4
    [0.1, 2.0, 3.5],   # точка 5
])

lat0_1, lon0_1, alt0_1 = 40.413552, -79.948717, 200.0

result_1 = compute_similarity_transform(gps_points_1, orb_points_1, lat0_1, lon0_1, alt0_1)
print("\nРезультат Similarity Transform (Набор 1):")
print(json.dumps(result_1, indent=2))


# ==================== НАБОР 2: Россия ====================
print("\n" + "=" * 60)
print("НАБОР 2: Россия (55.49N, 38.63E)")
print("=" * 60)

# GPS точки: lat lon; ORB: x y z
raw_data_2 = """55.49168528 38.63357190; -0.211100 -0.621200 1.104600
55.49197385 38.63378645; 0.395000 -0.783600 1.083500
55.49217129 38.63394737; 0.867600 -0.648700 1.014400
55.49169440 38.63432285; 0.664700 -0.312700 0.963500
55.49214396 38.63491288; 1.194800 -0.396100 1.028600"""

gps_points_2 = []
orb_points_2 = []

for line in raw_data_2.strip().split('\n'):
    gps_part, orb_part = line.split(';')
    lat, lon = map(float, gps_part.strip().split())
    x, y, z = map(float, orb_part.strip().split())
    gps_points_2.append([lat, lon, 0.0])  # высота не указана, берём 0
    orb_points_2.append([x, y, z])

gps_points_2 = np.array(gps_points_2)
orb_points_2 = np.array(orb_points_2)

lat0_2, lon0_2, alt0_2 = 55.49168528, 38.63357190, 0.0

result_2 = compute_similarity_transform(gps_points_2, orb_points_2, lat0_2, lon0_2, alt0_2)
print("\nРезультат Similarity Transform (Набор 2):")
print(json.dumps(result_2, indent=2))
