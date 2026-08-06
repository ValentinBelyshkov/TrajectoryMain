import os
import uuid
import json
import re
from pathlib import Path
from fastapi import APIRouter, HTTPException, UploadFile, File, Request
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
import aiofiles

from app.routers.projects import (
    get_projects_root,
    get_project_path,
    read_project_metadata,
    write_project_metadata,
)

router = APIRouter()

class CalibrationPointRequest(BaseModel):
    imageX: float
    imageY: float
    lat: float
    lng: float
    altitude: float

class GCPSaveRequest(BaseModel):
    image_filename: str
    points: List[CalibrationPointRequest]

class SingleImageGCP(BaseModel):
    image_filename: str
    points: List[CalibrationPointRequest]

class AllGCPSaveRequest(BaseModel):
    images: List[SingleImageGCP]

class CalibrationStatusResponse(BaseModel):
    project_id: str
    calibrated: bool
    calibration_file: Optional[str] = None

@router.post("/{project_id}/upload-image")
async def upload_calibration_image(request: Request, project_id: str, file: UploadFile = File(...)):
    """Upload a calibration image to the project folder."""
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    project_path = get_project_path(projects_root, project_id)
    calibrations_dir = project_path / "calibrations"
    calibrations_dir.mkdir(parents=True, exist_ok=True)
    
    # Use original filename for the calibration image
    save_path = calibrations_dir / file.filename
    
    with open(save_path, "wb") as f:
        content = await file.read()
        f.write(content)
    
    return {
        "success": True,
        "image_filename": file.filename,
        "image_url": f"/api/projects/{project_id}/calibrations/{file.filename}"
    }

@router.get("/{project_id}/calibrations/{image_name}")
async def get_calibration_image(request: Request, project_id: str, image_name: str):
    """Serve calibration images from the project folder."""
    projects_root = get_projects_root(request)
    project_path = get_project_path(projects_root, project_id)
    image_path = project_path / "calibrations" / image_name
    
    if not image_path.exists():
        raise HTTPException(status_code=404, detail="Image not found")
    
    # Determine content type based on extension
    ext = image_name.lower().split('.')[-1] if '.' in image_name else 'jpg'
    content_type = {
        'jpg': 'image/jpeg',
        'jpeg': 'image/jpeg',
        'png': 'image/png',
        'gif': 'image/gif',
        'webp': 'image/webp'
    }.get(ext, 'image/jpeg')
    
    async def file_iterator():
        async with aiofiles.open(image_path, 'rb') as f:
            while True:
                chunk = await f.read(8192)
                if not chunk:
                    break
                yield chunk
    
    return StreamingResponse(
        file_iterator(),
        media_type=content_type,
        headers={"Content-Disposition": f"inline; filename={image_name}"}
    )

@router.get("/{project_id}/procframe")
async def list_proc_frames(request: Request, project_id: str):
    """List all frames in the procframe directory of a project."""
    projects_root = get_projects_root(request)
    project_path = get_project_path(projects_root, project_id)
    proc_frames_dir = project_path / "procframe"
    
    if not proc_frames_dir.exists():
        return []
    
    frames = []
    # Supporting common image formats
    for ext in ['*.jpg', '*.jpeg', '*.png', '*.webp', '*.JPG', '*.JPEG', '*.PNG', '*.WEBP']:
        for frame_path in proc_frames_dir.glob(ext):
            frames.append({
                "filename": frame_path.name,
                "url": f"/api/projects/{project_id}/procframe/{frame_path.name}"
            })
    
    # Sort by filename
    frames.sort(key=lambda x: x["filename"])
    return frames

@router.get("/{project_id}/procframe/{image_name}")
async def get_proc_frame(request: Request, project_id: str, image_name: str):
    """Serve images from the procframe folder."""
    projects_root = get_projects_root(request)
    project_path = get_project_path(projects_root, project_id)
    image_path = project_path / "procframe" / image_name
    
    if not image_path.exists():
        raise HTTPException(status_code=404, detail="Image not found")
    
    ext = image_name.lower().split('.')[-1] if '.' in image_name else 'jpg'
    content_type = {
        'jpg': 'image/jpeg',
        'jpeg': 'image/jpeg',
        'png': 'image/png',
        'webp': 'image/webp'
    }.get(ext, 'image/jpeg')
    
    async def file_iterator():
        async with aiofiles.open(image_path, 'rb') as f:
            while True:
                chunk = await f.read(8192)
                if not chunk:
                    break
                yield chunk
    
    return StreamingResponse(
        file_iterator(),
        media_type=content_type,
        headers={"Content-Disposition": f"inline; filename={image_name}"}
    )

@router.post("/{project_id}/save-gcp")
async def save_gcp_file(request: Request, project_id: str, gcp_request: GCPSaveRequest):
    """Save GCP (Ground Control Points) file with +proj=utm header."""
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    if len(gcp_request.points) != 1:
        raise HTTPException(status_code=400, detail="Must provide exactly 1 point")
    
    project_path = get_project_path(projects_root, project_id)
    calibrations_dir = project_path / "calibrations"
    calibrations_dir.mkdir(parents=True, exist_ok=True)
    
    gcp_filename = f"{Path(gcp_request.image_filename).stem}.gpc"
    gcp_path = calibrations_dir / gcp_filename
    
    # Generate GCP file content with +proj=utm header
    gpc_content = f"+proj=utm +zone=37 +datum=WGS84\n"
    gpc_content += f"{gcp_request.image_filename}\n"
    gpc_content += f"{len(gcp_request.points)}\n"
    
    for point in gcp_request.points:
        # Format: x y lng lat altitude
        gpc_content += f"{point.imageX:.6f} {point.imageY:.6f} {point.lng:.6f} {point.lat:.6f} {point.altitude:.2f}\n"
    
    with open(gcp_path, "w", encoding="utf-8") as f:
        f.write(gpc_content)
    
    # Update project metadata
    project.calibration_status = "calibrated"
    write_project_metadata(projects_root, project)
    
    return {
        "success": True,
        "gcp_filename": gcp_filename,
        "calibration_status": "calibrated"
    }

@router.post("/{project_id}/save-all-gcp")
async def save_all_gcp(request: Request, project_id: str, save_request: AllGCPSaveRequest):
    """Save all GCP files and generate calib.txt by matching points in procframe txt files."""
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    project_path = get_project_path(projects_root, project_id)
    procframe_dir = project_path / "procframe"
    calibrations_dir = project_path / "calibrations"
    calibrations_dir.mkdir(parents=True, exist_ok=True)
    
    calib_lines = []
    
    for img_gcp in save_request.images:
        # 1. Save .gpc file as before
        gcp_filename = f"{Path(img_gcp.image_filename).stem}.gpc"
        gcp_path = calibrations_dir / gcp_filename
        
        gpc_content = f"+proj=utm +zone=37 +datum=WGS84\n"
        gpc_content += f"{img_gcp.image_filename}\n"
        gpc_content += f"{len(img_gcp.points)}\n"
        
        for point in img_gcp.points:
            gpc_content += f"{point.imageX:.6f} {point.imageY:.6f} {point.lng:.6f} {point.lat:.6f} {point.altitude:.2f}\n"
            
        with open(gcp_path, "w", encoding="utf-8") as f:
            f.write(gpc_content)
            
        # 2. Match points for calib.txt
        txt_path = procframe_dir / f"{Path(img_gcp.image_filename).stem}.txt"
        if txt_path.exists():
            try:
                with open(txt_path, 'r') as f:
                    content = f.readline().strip()
                    # Find all numbers in the first line
                    nums = [float(s) for s in re.findall(r"[-+]?\d*\.\d+|\d+", content)]
                    if len(nums) >= 3:
                        # Extract x, y, z from the numbers found (the camera pose)
                        x, y, z = nums[:3]
                        if img_gcp.points:
                            point = img_gcp.points[0] # Take the first point (1 point per image now)
                            # New format: pose(x, y, z) ; GPS(lat, lon)
                            calib_lines.append(f"{x:.6f} {y:.6f} {z:.6f}; {point.lat:.8f} {point.lng:.8f}\n")
            except Exception as e:
                print(f"Error reading {txt_path}: {e}")
                continue

    # Save calib.txt
    calib_txt_path = calibrations_dir / "calib.txt"
    with open(calib_txt_path, "w", encoding="utf-8") as f:
        f.writelines(calib_lines)
            
    # Update project metadata
    project.calibration_status = "calibrated"
    write_project_metadata(projects_root, project)
    
    return {
        "success": True,
        "calibration_status": "calibrated",
        "points_count": len(calib_lines)
    }

@router.get("/{project_id}/status")
async def get_calibration_status(request: Request, project_id: str):
    """Check if project has calibration."""
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    project_path = get_project_path(projects_root, project_id)
    calibrations_dir = project_path / "calibrations"
    
    if not calibrations_dir.exists():
        return {
            "project_id": project_id,
            "calibrated": False,
            "calibration_file": None
        }
    
    # Find any .gpc files
    gpc_files = list(calibrations_dir.glob("*.gpc"))
    
    if gpc_files:
        return {
            "project_id": project_id,
            "calibrated": True,
            "calibration_file": str(gpc_files[0])
        }
    
    return {
        "project_id": project_id,
        "calibrated": False,
        "calibration_file": None
    }

# Legacy endpoints for backward compatibility

@router.post("/start")
async def start_calibration(request: Request, project_id: str):
    """
    Simulates 15-second recording and returns 3 frame URLs.
    In production, this would connect to drone camera and record video.
    """
    calibration_path = request.app.state.calibration_path
    frames_dir = calibration_path / project_id
    frames_dir.mkdir(parents=True, exist_ok=True)
    
    frame_urls = []
    for i in range(3):
        frame_path = frames_dir / f"frame_{i}.jpg"
        
        # Create a simple placeholder image
        import numpy as np
        import cv2
        img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        cv2.imwrite(str(frame_path), img)
        
        frame_urls.append(f"/api/projects/calibration/frames/{project_id}/frame_{i}.jpg")
    
    return {
        "project_id": project_id,
        "frames": frame_urls,
        "duration": 15,
        "status": "completed"
    }

@router.get("/frames/{project_id}/{frame_name}")
async def get_frame(request: Request, project_id: str, frame_name: str):
    """Serve calibration frame images."""
    calibration_path = request.app.state.calibration_path
    frame_path = calibration_path / project_id / frame_name
    if not frame_path.exists():
        raise HTTPException(status_code=404, detail="Frame not found")
    
    async def file_iterator():
        async with aiofiles.open(frame_path, 'rb') as f:
            while True:
                chunk = await f.read(8192)
                if not chunk:
                    break
                yield chunk
    
    return StreamingResponse(
        file_iterator(),
        media_type="image/jpeg",
        headers={"Content-Disposition": f"inline; filename={frame_name}"}
    )


# Auto-calibration endpoints

class AutoCalibrationDownloadRequest(BaseModel):
    lat1: float
    lng1: float
    lat2: float
    lng2: float
    zoom: int

@router.post("/{project_id}/auto/download-geotiff")
async def download_geotiff(request: Request, project_id: str, req: AutoCalibrationDownloadRequest):
    """Download map tiles and create GeoTIFF for the specified region."""
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    project_path = get_project_path(projects_root, project_id)
    calibrations_dir = project_path / "calibrations"
    calibrations_dir.mkdir(parents=True, exist_ok=True)
    
    # Path to tms2geotiff script
    script_path = Path("/home/engine/project/scripts/tms2geotiff.py")
    output_path = calibrations_dir / "map_geotiff.tiff"
    
    # Build command
    cmd = [
        "python3", str(script_path),
        "-s", "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        "-f", f"{req.lat1},{req.lng1}",
        "-t", f"{req.lat2},{req.lng2}",
        "-z", str(req.zoom),
        str(output_path)
    ]
    
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=300  # 5 minutes timeout
        )
        
        if result.returncode != 0:
            return {
                "success": False,
                "error": result.stderr or "Failed to download map tiles",
                "output": result.stdout
            }
        
        return {
            "success": True,
            "geotiff_path": str(output_path),
            "message": "GeoTIFF successfully created"
        }
    except subprocess.TimeoutExpired:
        return {
            "success": False,
            "error": "Timeout downloading map tiles"
        }
    except Exception as e:
        return {
            "success": False,
            "error": str(e)
        }

@router.get("/{project_id}/auto/geotiff-status")
async def get_geotiff_status(request: Request, project_id: str):
    """Check if GeoTIFF exists for this project."""
    projects_root = get_projects_root(request)
    project_path = get_project_path(projects_root, project_id)
    geotiff_path = project_path / "calibrations" / "map_geotiff.tiff"
    
    return {
        "exists": geotiff_path.exists(),
        "path": str(geotiff_path) if geotiff_path.exists() else None
    }

@router.get("/{project_id}/auto/frames")
async def get_auto_calibration_frames(request: Request, project_id: str):
    """Get list of frames available for auto calibration."""
    projects_root = get_projects_root(request)
    project_path = get_project_path(projects_root, project_id)
    frames_dir = project_path / "frames"
    
    if not frames_dir.exists():
        # Try procframe
        frames_dir = project_path / "procframe"
    
    if not frames_dir.exists():
        return []
    
    frames = []
    for ext in ['*.jpg', '*.jpeg', '*.png', '*.JPG', '*.JPEG', '*.PNG']:
        for frame_path in frames_dir.glob(ext):
            frames.append({
                "filename": frame_path.name,
                "url": f"/api/projects/{project_id}/frames/{frame_path.name}"
            })
    
    frames.sort(key=lambda x: x["filename"])
    return frames

@router.post("/{project_id}/auto/match-image")
async def match_image_to_geotiff(request: Request, project_id: str, match_request: dict):
    """Match a drone image with the GeoTIFF map using geo_matcher.py."""
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    if not match_request or "image_filename" not in match_request:
        raise HTTPException(status_code=400, detail="image_filename is required")
    
    image_filename = match_request["image_filename"]
    
    project_path = get_project_path(projects_root, project_id)
    calibrations_dir = project_path / "calibrations"
    geotiff_path = calibrations_dir / "map_geotiff.tiff"
    
    if not geotiff_path.exists():
        raise HTTPException(status_code=400, detail="GeoTIFF not found. Please download map first.")
    
    # Find image in frames directory
    frames_dir = project_path / "frames"
    if not frames_dir.exists():
        frames_dir = project_path / "procframe"
    
    if not frames_dir.exists():
        raise HTTPException(status_code=400, detail="Frames directory not found")
    
    image_path = frames_dir / image_filename
    if not image_path.exists():
        raise HTTPException(status_code=404, detail=f"Image not found: {image_filename}")
    
    # Path to geo_matcher script
    script_path = Path("/home/engine/project/scripts/geo_matcher.py")
    
    # Build command
    cmd = [
        "python3", str(script_path),
        "--map", str(geotiff_path),
        "--image", str(image_path)
    ]
    
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=120  # 2 minutes timeout
        )
        
        output = result.stdout + result.stderr
        
        if result.returncode == 0 and "erfolgreich" in output.lower():
            # Update project calibration status
            project.calibration_status = "calibrated"
            write_project_metadata(projects_root, project)
            
            # Save calibration result
            calib_result = {
                "success": True,
                "image_filename": image_filename,
                "geotiff_path": str(geotiff_path),
                "output": output
            }
            with open(calibrations_dir / "auto_calib_result.json", "w") as f:
                json.dump(calib_result, f)
            
            return {
                "success": True,
                "message": "Калибровка успешно пройдена",
                "details": output
            }
        else:
            return {
                "success": False,
                "message": "Ошибка калибровки. Попробуйте другое фото или параметры снимка",
                "details": output
            }
    except subprocess.TimeoutExpired:
        return {
            "success": False,
            "message": "Timeout matching image"
        }
    except Exception as e:
        return {
            "success": False,
            "message": str(e)
        }
