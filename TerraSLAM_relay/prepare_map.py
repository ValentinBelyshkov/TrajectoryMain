#!/usr/bin/env python3
"""prepare_map.py — один раз перед полётом"""
import rasterio
import numpy as np
import cv2
import pickle

def prepare_map(geotiff_path, output_path='map_data.pkl'):
    ds = rasterio.open(geotiff_path)
    
    # Читаем всю карту (5×5 км должно влезть в RAM)
    img = ds.read()
    if img.shape[0] >= 3:
        rgb = np.dstack((img[0], img[1], img[2])).astype(np.uint8)
    else:
        g = img[0].astype(np.uint8)
        rgb = np.dstack((g, g, g))
    
    # Конвертируем в grayscale для matching
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    
    # Извлекаем ORB для всей карты (быстрее, чем каждый раз в полёте)
    orb = cv2.ORB_create(nfeatures=5000, scaleFactor=1.2, nlevels=8)
    kp_map, des_map = orb.detectAndCompute(gray, None)
    
    data = {
        'rgb': rgb,
        'gray': gray,
        'kp': kp_map,
        'des': des_map,
        'transform': ds.transform,  # affine: пиксели → координаты карты
        'crs': ds.crs,
        'shape': ds.shape,
        'bounds': ds.bounds,  # left, bottom, right, top
    }
    
    with open(output_path, 'wb') as f:
        pickle.dump(data, f)
    
    print(f"Map saved: {ds.shape} pixels")
    print(f"Bounds: {ds.bounds}")
    print(f"GSD: {ds.res[0]:.2f} m/px")
    print(f"ORB features: {len(kp_map)}")

if __name__ == '__main__':
    prepare_map('map_5x5.tif')
