#!/usr/bin/env python3
"""
tms2geotiff.py - Download tiles from tile servers and convert them to GeoTIFF

Based on gumblex/tms2geotiff
Usage: python3 tms2geotiff.py -s <tile_url> -f <lat1,lng1> -t <lat2,lng2> -z <zoom> <output>
Example: python3 tms2geotiff.py -s https://tile.openstreetmap.org/{z}/{x}/{y}.png -f 45.699,127 -t 30,148.492 -z 6 output.tiff
"""

import sys
import os
import math
import argparse
import tempfile
import shutil
from pathlib import Path
import urllib.request
import json

try:
    from osgeo import gdal, osr, ogr
except ImportError:
    print("Error: GDAL Python bindings not found. Install with: pip install GDAL")
    sys.exit(1)

class TileDownloader:
    def __init__(self, tile_url, zoom, lat1, lng1, lat2, lng2, output_path):
        self.tile_url = tile_url
        self.zoom = int(zoom)
        self.lat1 = min(float(lat1), float(lat2))
        self.lat2 = max(float(lat1), float(lat2))
        self.lng1 = min(float(lng1), float(lng2))
        self.lng2 = max(float(lng1), float(lng2))
        self.output_path = output_path
        
    def deg2num(self, lat, lng):
        lat_r = math.radians(lat)
        n = 2.0 ** self.zoom
        x = int((lng + 180.0) / 360.0 * n)
        y = int((1.0 - math.asinh(math.tan(lat_r)) / math.pi) / 2.0 * n)
        return x, y

    def download_tile(self, x, y):
        url = self.tile_url.replace('{z}', str(self.zoom))
        url = url.replace('{x}', str(x))
        url = url.replace('{y}', str(y))
        
        headers = {
            'User-Agent': 'Mozilla/5.0 (compatible; tms2geotiff/1.0)',
        }
        
        req = urllib.request.Request(url, headers=headers)
        try:
            with urllib.request.urlopen(req, timeout=30) as response:
                return response.read()
        except Exception as e:
            print(f"Error downloading tile {x},{y}: {e}")
            return None

    def download_tiles(self):
        x1, y1 = self.deg2num(self.lat2, self.lng1)
        x2, y2 = self.deg2num(self.lat1, self.lng2)
        
        x1 = max(0, min(x1, 2**self.zoom - 1))
        x2 = max(0, min(x2, 2**self.zoom - 1))
        y1 = max(0, min(y1, 2**self.zoom - 1))
        y2 = max(0, min(y2, 2**self.zoom - 1))
        
        print(f"Downloading tiles from zoom {self.zoom}, x: {x1}-{x2}, y: {y1}-{y2}")
        
        tiles = {}
        for x in range(min(x1, x2), max(x1, x2) + 1):
            for y in range(min(y1, y2), max(y1, y2) + 1):
                tile_data = self.download_tile(x, y)
                if tile_data:
                    tiles[(x, y)] = tile_data
        
        return tiles, x1, y1, x2, y2

    def create_geotiff(self, tiles, x1, y1, x2, y2):
        tile_size = 256
        width = (max(x1, x2) - min(x1, x2) + 1) * tile_size
        height = (max(y1, y2) - min(y1, y2) + 1) * tile_size
        
        # Calculate bounds in EPSG:4326
        lng_min, lat_max = self.deg2num(self.lat2, self.lng1)
        lng_max, lat_min = self.deg2num(self.lat1, self.lng2)
        
        # Tile coordinates to lat/lng
        def num2deg(x, y):
            n = 2.0 ** self.zoom
            lng = x / n * 360.0 - 180.0
            lat_r = math.atan(math.sinh(math.pi * (1 - 2*y/n)))
            lat = math.degrees(lat_r)
            return lat, lng
        
        lat_ul, lng_ul = num2deg(min(x1, x2), min(y1, y2))
        lat_lr, lng_lr = num2deg(max(x1, x2) + 1, max(y1, y2) + 1)
        
        print(f"Creating GeoTIFF with bounds: {lat_ul:.6f},{lng_ul:.6f} to {lat_lr:.6f},{lng_lr:.6f}")
        
        # Create VRT file for mosaic
        import tempfile
        with tempfile.NamedTemporaryFile(suffix='.vrt', delete=False) as vrt_file:
            vrt_path = vrt_file.name
        
        with open(vrt_path, 'w') as f:
            f.write(f'''<VRTDataset rasterXSize="{width}" rasterYSize="{height}">
  <SRS>EPSG:4326</SRS>
  <GeoTransform>{lng_ul}, {(lng_lr-lng_ul)/width}, 0, {lat_ul}, 0, {(lat_lr-lat_ul)/height}</GeoTransform>
''')
            
            tile_idx = 0
            for x in range(min(x1, x2), max(x1, x2) + 1):
                for y in range(min(y1, y2), max(y1, y2) + 1):
                    tile_x = (x - min(x1, x2)) * tile_size
                    tile_y = (y - min(y1, y2)) * tile_size
                    
                    if (x, y) in tiles:
                        # Save tile to temp file
                        with tempfile.NamedTemporaryFile(suffix='.png', delete=False) as tile_file:
                            tile_file.write(tiles[(x, y)])
                            tile_path = tile_file.name
                        
                        f.write(f'''  <VRTRasterBand dataType="Byte" band="1">
    <SimpleSource>
      <SourceFilename>{tile_path}</SourceFilename>
      <SourceBand>1</SourceBand>
      <SrcRect xOff="0" yOff="0" xSize="256" ySize="256"/>
      <DstRect xOff="{tile_x}" yOff="{tile_y}" xSize="256" ySize="256"/>
    </SimpleSource>
  </VRTRasterBand>
''')
                        tile_idx += 1
            
            f.write('</VRTDataset>')
        
        # Convert VRT to GeoTIFF
        driver = gdal.GetDriverByName('GTiff')
        vrt_ds = gdal.Open(vrt_path)
        
        if vrt_ds is None:
            raise RuntimeError("Failed to create VRT dataset")
        
        output_ds = driver.CreateCopy(self.output_path, vrt_ds, options=['TIFFTYPE=PIXEL'])
        
        if output_ds is None:
            raise RuntimeError("Failed to create GeoTIFF")
        
        # Cleanup
        vrt_ds = None
        gdal.Unlink(vrt_path)
        
        # Remove temp tile files
        for x in range(min(x1, x2), max(x1, x2) + 1):
            for y in range(min(y1, y2), max(y1, y2) + 1):
                if (x, y) in tiles:
                    temp_tile = tempfile.gettempdir() + '/' + os.path.basename(tempfile.NamedTemporaryFile(suffix='.png', delete=False).name)
        
        print(f"GeoTIFF saved to: {self.output_path}")
        return True

    def run(self):
        print(f"Downloading map tiles for region ({self.lat1:.4f},{self.lng1:.4f}) to ({self.lat2:.4f},{self.lng2:.4f})")
        
        tiles, x1, y1, x2, y2 = self.download_tiles()
        
        if not tiles:
            print("Error: No tiles downloaded")
            return False
        
        self.create_geotiff(tiles, x1, y1, x2, y2)
        
        print("Done!")
        return True

def main():
    parser = argparse.ArgumentParser(description='Download tiles and convert to GeoTIFF')
    parser.add_argument('-s', '--source', required=True, help='Tile source URL with {z}/{x}/{y} placeholders')
    parser.add_argument('-f', '--from-corner', required=True, help='First corner: lat,lng')
    parser.add_argument('-t', '--to-corner', required=True, help='Second corner: lat,lng')
    parser.add_argument('-z', '--zoom', required=True, help='Zoom level')
    parser.add_argument('output', help='Output GeoTIFF file path')
    
    args = parser.parse_args()
    
    lat1, lng1 = args.from_corner.split(',')
    lat2, lng2 = args.to_corner.split(',')
    
    downloader = TileDownloader(
        tile_url=args.source,
        zoom=args.zoom,
        lat1=lat1,
        lng1=lng1,
        lat2=lat2,
        lng2=lng2,
        output_path=args.output
    )
    
    success = downloader.run()
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
