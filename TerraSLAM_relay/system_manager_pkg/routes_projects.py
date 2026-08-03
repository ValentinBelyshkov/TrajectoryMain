import asyncio
import json
import os
import shutil
import time
import uuid
from pathlib import Path
from typing import Optional, List
from datetime import datetime

from fastapi import APIRouter, HTTPException, UploadFile, File, Request
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from fastapi.encoders import jsonable_encoder

router = APIRouter()

def get_video_duration(input_file):
    try:
        import subprocess
        result = subprocess.run([
            "ffprobe", "-v", "error", "-show_entries", "format=duration",
            "-of", "default=noprint_wrappers=1:nokey=1", input_file
        ], capture_output=True, text=True, check=True)
        return float(result.stdout.strip())
    except Exception as e:
        print(f"Error getting duration: {e}")
        return 0

async def process_video_with_progress(project_id, input_file, output_pattern):
    duration = get_video_duration(input_file)
    start_time = time.time()
    
    process = await asyncio.create_subprocess_exec(
        "ffmpeg", "-y", "-i", input_file, 
        "-vf", "fps=10", 
        "-q:v", "2", 
        "-progress", "pipe:1",
        output_pattern,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )

    while True:
        line = await process.stdout.readline()
        if not line:
            break
        
        line_str = line.decode().strip()
        if "out_time_us=" in line_str:
            try:
                us = int(line_str.split('=')[1])
                progress = (us / 1000000) / duration if duration > 0 else 0
                progress = min(progress, 0.99)
                
                elapsed_time = time.time() - start_time
                if progress > 0.01:
                    total_estimated_time = elapsed_time / progress
                    remaining_time = total_estimated_time - elapsed_time
                else:
                    remaining_time = 0
                
                from .telemetry import telemetry_manager
                await telemetry_manager.broadcast(project_id, {
                    "type": "ffmpeg_progress",
                    "progress": progress,
                    "remaining_time": round(remaining_time, 1)
                })
            except (ValueError, IndexError):
                pass
                
    await process.wait()
    
    from .telemetry import telemetry_manager
    await telemetry_manager.broadcast(project_id, {
        "type": "ffmpeg_progress",
        "progress": 1.0,
        "remaining_time": 0
    })

def get_projects_root(request: Request) -> Path:
    """Get projects root path from app state."""
    return request.app.state.projects_path

class ProjectBase(BaseModel):
    name: str
    type: str

class ProjectCreate(ProjectBase):
    video_filename: Optional[str] = None
    frames_path: Optional[str] = None

class Project(ProjectBase):
    id: str
    created_at: datetime
    video_filename: Optional[str] = None
    frames_path: Optional[str] = None
    calibration_status: str = "not_calibrated"

def ensure_projects_directory(projects_root: Path):
    """Ensure the projects directory exists."""
    projects_root.mkdir(parents=True, exist_ok=True)

def get_project_path(projects_root: Path, project_id: str) -> Path:
    """Get the path to a project directory."""
    return projects_root / project_id

def get_metadata_path(projects_root: Path, project_id: str) -> Path:
    """Get the path to a project's metadata.json file."""
    return get_project_path(projects_root, project_id) / "metadata.json"

def read_project_metadata(projects_root: Path, project_id: str) -> Optional[Project]:
    """Read and validate project metadata from JSON file."""
    metadata_path = get_metadata_path(projects_root, project_id)
    if not metadata_path.exists():
        return None
    
    try:
        with open(metadata_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return Project.model_validate(data)
    except Exception:
        return None

def write_project_metadata(projects_root: Path, project: Project) -> None:
    """Write project metadata to JSON file and create project structure."""
    project_path = get_project_path(projects_root, project.id)
    project_path.mkdir(parents=True, exist_ok=True)
    
    (project_path / "logs").mkdir(exist_ok=True)
    (project_path / "calibrations").mkdir(exist_ok=True)
    (project_path / "photos").mkdir(exist_ok=True)
    (project_path / "procframe").mkdir(exist_ok=True)
    
    metadata_path = get_metadata_path(projects_root, project.id)
    with open(metadata_path, "w", encoding="utf-8") as f:
        json.dump(jsonable_encoder(project), f, indent=2, ensure_ascii=False)

@router.get("", response_model=List[Project])
async def get_projects(request: Request):
    projects_root = get_projects_root(request)
    ensure_projects_directory(projects_root)
    projects = []
    
    if projects_root.exists():
        for project_dir in projects_root.iterdir():
            if project_dir.is_dir():
                project = read_project_metadata(projects_root, project_dir.name)
                if project:
                    projects.append(project)
    
    return projects

@router.post("", response_model=Project)
async def create_project(request: Request, project: ProjectCreate):
    projects_root = get_projects_root(request)
    ensure_projects_directory(projects_root)
    
    new_project = Project(
        id=str(uuid.uuid4()),
        name=project.name,
        type=project.type,
        created_at=datetime.now(),
        video_filename=project.video_filename,
        calibration_status="not_calibrated"
    )
    
    write_project_metadata(projects_root, new_project)
    return new_project

@router.get("/{project_id}", response_model=Project)
async def get_project(request: Request, project_id: str):
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    return project

@router.put("/{project_id}", response_model=Project)
async def update_project(request: Request, project_id: str, project: ProjectBase):
    projects_root = get_projects_root(request)
    existing_project = read_project_metadata(projects_root, project_id)
    
    if existing_project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    existing_project.name = project.name
    existing_project.type = project.type
    
    write_project_metadata(projects_root, existing_project)
    return existing_project

@router.delete("/{project_id}")
async def delete_project(request: Request, project_id: str):
    projects_root = get_projects_root(request)
    project_path = get_project_path(projects_root, project_id)
    
    if not project_path.exists():
        raise HTTPException(status_code=404, detail="Project not found")
    
    shutil.rmtree(project_path)
    return {"message": "Project deleted"}

@router.post("/{project_id}/video")
async def upload_video(request: Request, project_id: str, file: UploadFile = File(...)):
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    project_path = get_project_path(projects_root, project_id)
    videos_dir = project_path / "videos"
    videos_dir.mkdir(parents=True, exist_ok=True)
    
    save_path = videos_dir / file.filename
    
    with open(save_path, "wb") as f:
        content = await file.read()
        f.write(content)
    
    project.video_filename = str(save_path)
    
    frames_dir = project_path / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    
    for old_frame in frames_dir.glob("*.jpg"):
        old_frame.unlink()

    input_file = str(save_path)
    output_pattern = str(frames_dir / "%04d.jpg")
    
    try:
        await process_video_with_progress(project_id, input_file, output_pattern)
        project.frames_path = str(frames_dir)
    except Exception as e:
        print(f"FFmpeg error: {e}")
    
    write_project_metadata(projects_root, project)
    
    return {"message": "Video uploaded and processed", "filename": str(save_path), "frames_path": project.frames_path}

@router.get("/{project_id}/frames")
async def get_project_frames(request: Request, project_id: str):
    """Get list of frames for a project."""
    projects_root = get_projects_root(request)
    project = read_project_metadata(projects_root, project_id)
    
    if project is None:
        raise HTTPException(status_code=404, detail="Project not found")
    
    project_path = get_project_path(projects_root, project_id)
    frames_dir = project_path / "frames"
    
    if not frames_dir.exists():
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

@router.get("/{project_id}/frames/{frame_name}")
async def get_project_frame(request: Request, project_id: str, frame_name: str):
    """Serve a frame image from the project."""
    import aiofiles
    
    projects_root = get_projects_root(request)
    project_path = get_project_path(projects_root, project_id)
    
    frame_path = project_path / "frames" / frame_name
    if not frame_path.exists():
        frame_path = project_path / "procframe" / frame_name
    
    if not frame_path.exists():
        raise HTTPException(status_code=404, detail="Frame not found")
    
    ext = frame_name.lower().split('.')[-1] if '.' in frame_name else 'jpg'
    content_type = {
        'jpg': 'image/jpeg',
        'jpeg': 'image/jpeg',
        'png': 'image/png',
        'webp': 'image/webp'
    }.get(ext, 'image/jpeg')
    
    async def file_iterator():
        async with aiofiles.open(frame_path, 'rb') as f:
            while True:
                chunk = await f.read(8192)
                if not chunk:
                    break
                yield chunk
    
    from fastapi.responses import StreamingResponse
    return StreamingResponse(
        file_iterator(),
        media_type=content_type,
        headers={"Content-Disposition": f"inline; filename={frame_name}"}
    )
