import os
import re
import subprocess
import time
import uuid
import json
from pathlib import Path
from fastapi import APIRouter, HTTPException, UploadFile, File, Form, Request
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
import aiofiles

router = APIRouter()

SESSIONS_DIR = Path(os.getenv("CALIBRATION_SESSIONS_DIR", "/home/orb/Database/calibration_sessions"))
SESSIONS_DIR.mkdir(parents=True, exist_ok=True)

PROCFRAME_DIR = Path(os.getenv("PROCFRAME_DIR", "/opt/main/Trajectory/output/procframe"))
PROJECTS_DIR = Path(os.getenv("PROJECTS_DIR", "/home/orb/Database/projects"))


class SessionStatus(str):
    RECORDING = "recording"
    TRIMMING = "trimming"
    PROCESSING = "processing"
    CORRELATING = "correlating"
    DONE = "done"
    ERROR = "error"


class TrimReq(BaseModel):
    segments: List[dict]


class ProcessReq(BaseModel):
    session_id: str
    project_id: Optional[str] = None


class CorrelatePoint(BaseModel):
    frame_idx: int
    pixel_x: float
    pixel_y: float
    lat: float
    lon: float
    alt: float


class CorrelateReq(BaseModel):
    points: List[CorrelatePoint]


def _session_path(session_id: str) -> Path:
    return SESSIONS_DIR / f"{session_id}.json"


def _load_session(session_id: str) -> dict:
    path = _session_path(session_id)
    if not path.exists():
        raise HTTPException(404, detail=f"Session {session_id} not found")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _save_session(session: dict):
    path = _session_path(session["id"])
    with open(path, "w", encoding="utf-8") as f:
        json.dump(session, f, ensure_ascii=False, indent=2)


def _new_session_id() -> str:
    return f"calib_{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"


# Legacy endpoints for backward compatibility (mounted under /api/projects)

@router.post("/{project_id}/upload-image")
async def upload_calibration_image(request: Request, project_id: str, file: UploadFile = File(...)):
    """Upload a calibration image to the project folder."""
    from .routes_projects import get_projects_root, get_project_path, read_project_metadata, write_project_metadata
    
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    project_path = get_project_path(projects_root, project_id)
    calibrations_dir = project_path / "calibrations"
    calibrations_dir.mkdir(parents=True, exist_ok=True)
    
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
    from .routes_projects import get_projects_root, get_project_path
    
    projects_root = get_projects_root(request)
    project_path = get_project_path(projects_root, project_id)
    image_path = project_path / "calibrations" / image_name
    
    if not image_path.exists():
        raise HTTPException(status_code=404, detail="Image not found")
    
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
    from .routes_projects import get_projects_root, get_project_path
    
    projects_root = get_projects_root(request)
    project_path = get_project_path(projects_root, project_id)
    proc_frames_dir = project_path / "procframe"
    
    if not proc_frames_dir.exists():
        return []
    
    frames = []
    for ext in ['*.jpg', '*.jpeg', '*.png', '*.webp', '*.JPG', '*.JPEG', '*.PNG', '*.WEBP']:
        for frame_path in proc_frames_dir.glob(ext):
            frames.append({
                "filename": frame_path.name,
                "url": f"/api/projects/{project_id}/procframe/{frame_path.name}"
            })
    
    frames.sort(key=lambda x: x["filename"])
    return frames


@router.get("/{project_id}/procframe/{image_name}")
async def get_proc_frame(request: Request, project_id: str, image_name: str):
    """Serve images from the procframe folder."""
    from .routes_projects import get_projects_root, get_project_path
    
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
async def save_gcp_file(request: Request, project_id: str, gcp_request: dict):
    """Save GCP (Ground Control Points) file with +proj=utm header."""
    from .routes_projects import get_projects_root, get_project_path, read_project_metadata, write_project_metadata
    
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    points = gcp_request.get("points", [])
    if len(points) != 1:
        raise HTTPException(status_code=400, detail="Must provide exactly 1 point")
    
    project_path = get_project_path(projects_root, project_id)
    calibrations_dir = project_path / "calibrations"
    calibrations_dir.mkdir(parents=True, exist_ok=True)
    
    image_filename = gcp_request.get("image_filename", "image.jpg")
    gcp_filename = f"{Path(image_filename).stem}.gpc"
    gcp_path = calibrations_dir / gcp_filename
    
    gpc_content = f"+proj=utm +zone=37 +datum=WGS84\n"
    gpc_content += f"{image_filename}\n"
    gpc_content += f"{len(points)}\n"
    
    for point in points:
        gpc_content += f"{point['imageX']:.6f} {point['imageY']:.6f} {point['lng']:.6f} {point['lat']:.6f} {point['altitude']:.2f}\n"
    
    with open(gcp_path, "w", encoding="utf-8") as f:
        f.write(gpc_content)
    
    project.calibration_status = "calibrated"
    write_project_metadata(projects_root, project)
    
    return {
        "success": True,
        "gcp_filename": gcp_filename,
        "calibration_status": "calibrated"
    }


@router.post("/{project_id}/save-all-gcp")
async def save_all_gcp(request: Request, project_id: str, save_request: dict):
    """Save all GCP files and generate calib.txt by matching points in procframe txt files."""
    from .routes_projects import get_projects_root, get_project_path, read_project_metadata, write_project_metadata
    
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    project_path = get_project_path(projects_root, project_id)
    procframe_dir = project_path / "procframe"
    calibrations_dir = project_path / "calibrations"
    calibrations_dir.mkdir(parents=True, exist_ok=True)
    
    calib_lines = []
    images = save_request.get("images", [])
    
    for img_gcp in images:
        image_filename = img_gcp.get("image_filename", "")
        gcp_filename = f"{Path(image_filename).stem}.gpc"
        gcp_path = calibrations_dir / gcp_filename
        
        points = img_gcp.get("points", [])
        gpc_content = f"+proj=utm +zone=37 +datum=WGS84\n"
        gpc_content += f"{image_filename}\n"
        gpc_content += f"{len(points)}\n"
        
        for point in points:
            gpc_content += f"{point['imageX']:.6f} {point['imageY']:.6f} {point['lng']:.6f} {point['lat']:.6f} {point['altitude']:.2f}\n"
            
        with open(gcp_path, "w", encoding="utf-8") as f:
            f.write(gpc_content)
            
        txt_path = procframe_dir / f"{Path(image_filename).stem}.txt"
        if txt_path.exists():
            try:
                with open(txt_path, 'r') as f:
                    content = f.readline().strip()
                    nums = [float(s) for s in re.findall(r"[-+]?\d*\.\d+|\d+", content)]
                    if len(nums) >= 3:
                        x, y, z = nums[:3]
                        if points:
                            point = points[0]
                            calib_lines.append(f"{x:.6f} {y:.6f} {z:.6f}; {point['lat']:.8f} {point['lng']:.8f}\n")
            except Exception as e:
                print(f"Error reading {txt_path}: {e}")
                continue

    calib_txt_path = calibrations_dir / "calib.txt"
    with open(calib_txt_path, "w", encoding="utf-8") as f:
        f.writelines(calib_lines)
        
    project.calibration_status = "calibrated"
    write_project_metadata(projects_root, project)
    
    return {
        "success": True,
        "calibration_status": "calibrated",
        "points_count": len(calib_lines)
    }


@router.get("/{project_id}/calibration/status")
async def get_calibration_status(request: Request, project_id: str):
    """Check if project has calibration."""
    from .routes_projects import get_projects_root, get_project_path, read_project_metadata
    
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
