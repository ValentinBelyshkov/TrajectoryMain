#!/usr/bin/env python3
"""
geo_matcher.py - Match drone images with a GeoTIFF map using OpenCV

This script compares a drone image with a geographic map (GeoTIFF) and finds
the transformation parameters to georeference the drone image.

Usage: python3 geo_matcher.py --map <geotiff_path> --image <drone_image_path> [--output <output_path>]
"""

import sys
import os
import cv2
import numpy as np
import argparse

try:
    from osgeo import gdal
except ImportError:
    print("Error: GDAL Python bindings not found. Install with: pip install GDAL")
    sys.exit(1)

class GeoMatcher:
    def __init__(self, map_path, image_path, output_path=None):
        self.map_path = map_path
        self.image_path = image_path
        self.output_path = output_path
        
    def load_geotiff(self):
        """Load GeoTIFF and extract geographic bounds."""
        ds = gdal.Open(self.map_path)
        if ds is None:
            raise RuntimeError(f"Failed to open GeoTIFF: {self.map_path}")
        
        self.geo_transform = ds.GetGeoTransform()
        width = ds.RasterXSize
        height = ds.RasterYSize
        
        # Read RGB bands
        band_r = ds.GetRasterBand(1).ReadAsArray()
        band_g = ds.GetRasterBand(2).ReadAsArray()
        band_b = ds.GetRasterBand(3).ReadAsArray()
        
        if band_r is None:
            band_r = ds.GetRasterBand(1).ReadAsArray()
            band_g = band_r
            band_b = band_r
        
        # Create RGB image
        map_image = np.zeros((height, width, 3), dtype=np.uint8)
        if len(band_r.shape) == 2:
            map_image[:, :, 0] = band_r
            if band_g is not None:
                map_image[:, :, 1] = band_g
            if band_b is not None:
                map_image[:, :, 2] = band_b
        else:
            map_image = np.stack([band_r, band_g, band_b], axis=2)
        
        # Calculate geographic bounds
        x_origin = self.geo_transform[0]
        y_origin = self.geo_transform[3]
        x_pixel = self.geo_transform[1]
        y_pixel = self.geo_transform[5]
        
        # Convert pixel to geographic coordinates
        lon_min = x_origin
        lon_max = x_origin + width * x_pixel
        lat_max = y_origin
        lat_min = y_origin + height * y_pixel
        
        self.map_georef = {
            'lon_min': lon_min, 'lon_max': lon_max,
            'lat_min': lat_min, 'lat_max': lat_max,
            'width': width, 'height': height
        }
        
        print(f"GeoTIFF bounds: lon [{lon_min:.6f}, {lon_max:.6f}], lat [{lat_min:.6f}, {lat_max:.6f}]")
        print(f"Image size: {width}x{height}")
        
        return map_image
    
    def pixel_to_geo(self, x, y):
        """Convert pixel coordinates to geographic coordinates."""
        gt = self.geo_transform
        lon = gt[0] + x * gt[1] + y * gt[2]
        lat = gt[3] + x * gt[4] + y * gt[5]
        return lat, lon
    
    def geo_to_pixel(self, lat, lon):
        """Convert geographic coordinates to pixel coordinates."""
        gt = self.geo_transform
        det = gt[1] * gt[5] - gt[2] * gt[4]
        if abs(det) < 1e-10:
            return None, None
        
        x = (gt[5] * (lon - gt[0]) - gt[2] * (lat - gt[3])) / det
        y = (-gt[4] * (lon - gt[0]) + gt[1] * (lat - gt[3])) / det
        return x, y
    
    def load_drone_image(self):
        """Load drone image."""
        img = cv2.imread(self.image_path)
        if img is None:
            raise RuntimeError(f"Failed to open image: {self.image_path}")
        print(f"Drone image size: {img.shape[1]}x{img.shape[0]}")
        return img
    
    def match_images(self, map_img, drone_img):
        """Match drone image with map using feature matching."""
        # Resize map for faster processing
        map_h, map_w = map_img.shape[:2]
        max_size = 2000
        if max(map_h, map_w) > max_size:
            scale = max_size / max(map_h, map_w)
            map_img = cv2.resize(map_img, None, fx=scale, fy=scale)
            print(f"Map resized to {map_img.shape[1]}x{map_img.shape[0]}")
        
        # Convert to grayscale
        if len(map_img.shape) == 3:
            map_gray = cv2.cvtColor(map_img, cv2.COLOR_BGR2GRAY)
        else:
            map_gray = map_img
            
        if len(drone_img.shape) == 3:
            drone_gray = cv2.cvtColor(drone_img, cv2.COLOR_BGR2GRAY)
        else:
            drone_gray = drone_img
        
        # Feature detection
        detector = cv2.SIFT_create(nfeatures=5000)
        
        print("Detecting features in map...")
        kp1, des1 = detector.detectAndCompute(map_gray, None)
        
        print("Detecting features in drone image...")
        kp2, des2 = detector.detectAndCompute(drone_gray, None)
        
        if des1 is None or des2 is None:
            print("Warning: Could not detect features in one of the images")
            return None, None, 0
        
        print(f"Features found - Map: {len(kp1)}, Drone: {len(kp2)}")
        
        # Feature matching
        bf = cv2.BFMatcher(cv2.NORM_L2)
        matches = bf.knnMatch(des1, des2, k=2)
        
        # Lowe's ratio test
        good_matches = []
        for m_n in matches:
            if len(m_n) == 2:
                m, n = m_n
                if m.distance < 0.7 * n.distance:
                    good_matches.append(m)
        
        print(f"Good matches after ratio test: {len(good_matches)}")
        
        if len(good_matches) < 10:
            print("Not enough matches found")
            return None, None, len(good_matches)
        
        # Get matched points
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches])
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches])
        
        # Find homography with RANSAC
        M, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
        
        if M is None:
            print("Could not find homography")
            return None, None, len(good_matches)
        
        inliers = mask.ravel().sum()
        print(f"Inliers: {inliers}/{len(good_matches)} ({100*inliers/len(good_matches):.1f}%)")
        
        if inliers < 10:
            print("Not enough inliers")
            return M, mask, inliers
        
        # Calculate geographic transformation
        h, w = drone_img.shape[:2]
        corners = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
        transformed = cv2.perspectiveTransform(corners, M)
        
        geo_corners = []
        for pt in transformed:
            lat, lon = self.pixel_to_geo(pt[0][0], pt[0][1])
            geo_corners.append({'lat': lat, 'lon': lon})
        
        result = {
            'success': True,
            'inliers': int(inliers),
            'total_matches': len(good_matches),
            'corners': geo_corners,
            'homography': M.tolist()
        }
        
        print("\nGeoreferenced corners:")
        for i, corner in enumerate(geo_corners):
            print(f"  Corner {i+1}: lat={corner['lat']:.6f}, lon={corner['lon']:.6f}")
        
        return M, mask, inliers
    
    def run(self):
        """Execute the geo matching process."""
        print(f"Loading GeoTIFF: {self.map_path}")
        map_img = self.load_geotiff()
        
        print(f"\nLoading drone image: {self.image_path}")
        drone_img = self.load_drone_image()
        
        print("\nMatching images...")
        M, mask, inliers = self.match_images(map_img, drone_img)
        
        if M is not None and inliers >= 10:
            print("\n✅ Calibrierung erfolgreich abgeschlossen!")
            print(f"Found {inliers} matching points between drone image and map")
            return True
        else:
            print("\n❌ Calibrirung fehlgeschlagen!")
            print("Bitte versuchen Sie ein anderes Foto oder andere Kartenparameter")
            return False

def main():
    parser = argparse.ArgumentParser(description='Match drone images with GeoTIFF maps')
    parser.add_argument('--map', required=True, help='Path to GeoTIFF map file')
    parser.add_argument('--image', required=True, help='Path to drone image')
    parser.add_argument('--output', help='Output file for transformation matrix')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.map):
        print(f"Error: Map file not found: {args.map}")
        sys.exit(1)
    
    if not os.path.exists(args.image):
        print(f"Error: Image file not found: {args.image}")
        sys.exit(1)
    
    matcher = GeoMatcher(args.map, args.image, args.output)
    success = matcher.run()
    
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
