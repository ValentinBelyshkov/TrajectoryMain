"""
Calibration workflow routes:
- Session management (create, list, get)
- Video recording (chunk upload, finalize)
- Video trimming (ffmpeg)
- SLAM processing on trimmed video (frame extraction + SLAM run + pose capture)
- 5-point correlation (similarity transform)
- Finalize calibration (write calib.gpc)
"""
import asyncio
import glob
import json
import os
import subprocess
import time
import uuid
from pathlib import Path
from typing import List, Optional

from fastapi import APIRouter, HTTPException, UploadFile, File, Form
from fastapi.responses import FileResponse
from pydantic import BaseModel

from .config import RunReq
from .process_manager import manager
from .ros_utils import ros2_run

router = APIRouter()

SESSIONS_DIR = Path(os.getenv("CALIBRATION_SESSIONS_DIR", "/opt/main/Trajectory/Database/projects/calibration_sessions"))
SESSIONS_DIR.mkdir(parents=True, exist_ok=True)

PROCFRAME_DIR = Path(os.getenv("PROCFRAME_DIR", "/opt/main/Trajectory/output/procframe"))
PROJECTS_DIR = Path(os.getenv("PROJECTS_DIR", "/opt/main/Trajectory/Database/projects"))


class SessionStatus(str):
    RECORDING = "recording"
    TRIMMING = "trimming"
    PROCESSING = "processing"
    CORRELATING = "correlating"
    DONE = "done"
    ERROR = "error"


class TrimReq(BaseModel):
    segments: List[dict]  # [{"start": 5.2, "end": 45.8}, ...]


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


# ========== SESSION CRUD ==========

@router.post("/api/v1/calibration/video/start")
async def start_recording():
    """Create a new calibration recording session."""
    session_id = _new_session_id()
    video_path = SESSIONS_DIR / f"{session_id}_raw.mp4"

    session = {
        "id": session_id,
        "created_at": int(time.time()),
        "status": SessionStatus.RECORDING,
        "video_path": str(video_path),
        "trimmed_video_path": None,
        "trim_segments": [],
        "project_id": session_id,
        "frames_dir": None,
        "procframe_dir": str(PROCFRAME_DIR),
        "frame_pose_data": [],
        "correlation_points": [],
        "transform": None,
        "calib_gpc_path": None,
    }
    _save_session(session)

    # Ensure video file exists (will be appended to via chunks)
    video_path.touch()

    return {"success": True, "session": session}


@router.post("/api/v1/calibration/video/start-from-project/{project_id}")
async def start_recording_from_project(project_id: str):
    """Create a new calibration recording session from an existing project video."""
    project_path = PROJECTS_DIR / project_id
    if not project_path.exists():
        raise HTTPException(404, detail=f"Project {project_id} not found")

    metadata_path = project_path / "metadata.json"
    if not metadata_path.exists():
        raise HTTPException(404, detail="Project metadata not found")

    with open(metadata_path, "r", encoding="utf-8") as f:
        project_data = json.load(f)

    video_filename = project_data.get("video_filename")
    if not video_filename:
        raise HTTPException(400, detail="Project has no video file")

    source_path = Path(video_filename)
    if not source_path.exists():
        raise HTTPException(404, detail=f"Project video not found: {video_filename}")

    session_id = _new_session_id()
    video_path = SESSIONS_DIR / f"{session_id}_raw.mp4"

    session = {
        "id": session_id,
        "created_at": int(time.time()),
        "status": SessionStatus.TRIMMING,
        "video_path": str(video_path),
        "trimmed_video_path": None,
        "trim_segments": [],
        "project_id": project_id,
        "frames_dir": None,
        "procframe_dir": str(PROCFRAME_DIR),
        "frame_pose_data": [],
        "correlation_points": [],
        "transform": None,
        "calib_gpc_path": None,
    }

    import shutil
    shutil.copy2(source_path, video_path)

    _save_session(session)
    return {"success": True, "session": session}


@router.post("/api/v1/calibration/video/{session_id}/chunk")
async def upload_chunk(session_id: str, chunk: UploadFile = File(...)):
    """Append a video chunk (webm/mp4) to the session's raw video file."""
    session = _load_session(session_id)
    if session["status"] != SessionStatus.RECORDING:
        raise HTTPException(400, detail=f"Session not in recording state: {session['status']}")

    video_path = Path(session["video_path"])
    content = await chunk.read()
    with open(video_path, "ab") as f:
        f.write(content)

    return {"success": True, "bytes_written": len(content)}


@router.post("/api/v1/calibration/video/{session_id}/stop")
async def stop_recording(session_id: str):
    """Finalize recording, ready for trimming."""
    session = _load_session(session_id)
    if session["status"] != SessionStatus.RECORDING:
        raise HTTPException(400, detail=f"Session not in recording state: {session['status']}")

    session["status"] = SessionStatus.TRIMMING
    _save_session(session)

    return {"success": True, "session": session}


@router.get("/api/v1/calibration/sessions")
async def list_sessions():
    """List all calibration sessions."""
    sessions = []
    for path in SESSIONS_DIR.glob("*.json"):
        try:
            with open(path, "r", encoding="utf-8") as f:
                s = json.load(f)
            sessions.append({
                "id": s["id"],
                "created_at": s["created_at"],
                "status": s["status"],
                "video_path": s["video_path"],
                "trimmed_video_path": s["trimmed_video_path"],
            })
        except Exception:
            pass
    sessions.sort(key=lambda x: x["created_at"], reverse=True)
    return {"success": True, "sessions": sessions}


@router.get("/api/v1/calibration/session/{session_id}")
async def get_session(session_id: str):
    """Get full session data."""
    session = _load_session(session_id)
    return {"success": True, "session": session}


# ========== VIDEO TRIMMING ==========

@router.post("/api/v1/calibration/video/{session_id}/trim")
async def trim_video(session_id: str, req: TrimReq):
    """Apply trim segments to raw video using ffmpeg."""
    session = _load_session(session_id)
    if session["status"] not in (SessionStatus.TRIMMING, SessionStatus.PROCESSING):
        raise HTTPException(400, detail=f"Session not in trimmable state: {session['status']}")

    raw_path = Path(session["video_path"])
    if not raw_path.exists():
        raise HTTPException(404, detail="Raw video not found")

    trimmed_path = SESSIONS_DIR / f"{session_id}_trimmed.mp4"

    # Build ffmpeg filter for multiple segments
    # Using concat demuxer with segment list
    segments = req.segments
    if not segments:
        raise HTTPException(400, detail="No trim segments provided")

    # Create a temp file list for concat
    list_path = SESSIONS_DIR / f"{session_id}_segments.txt"
    with open(list_path, "w") as f:
        for seg in segments:
            start = seg["start"]
            end = seg["end"]
            duration = end - start
            if duration <= 0:
                continue
            f.write(f"file '{raw_path.as_posix()}'\n")
            f.write(f"inpoint {start}\n")
            f.write(f"outpoint {end}\n")

    # Run ffmpeg
    cmd = [
        "ffmpeg", "-y", "-f", "concat", "-safe", "0",
        "-i", str(list_path),
        "-c", "copy", str(trimmed_path)
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        # Fallback: re-encode if copy fails
        cmd = [
            "ffmpeg", "-y", "-f", "concat", "-safe", "0",
            "-i", str(list_path),
            "-c:v", "libx264", "-preset", "fast", "-crf", "23",
            str(trimmed_path)
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            raise HTTPException(500, detail=f"ffmpeg trim failed: {result.stderr}")

    list_path.unlink(missing_ok=True)

    session["trimmed_video_path"] = str(trimmed_path)
    session["trim_segments"] = segments
    session["status"] = SessionStatus.PROCESSING
    _save_session(session)

    return {"success": True, "session": session, "trimmed_path": str(trimmed_path)}


# ========== SLAM PROCESSING ==========

@router.post("/api/v1/calibration/process")
async def process_calibration(req: ProcessReq):
    """
    Process trimmed video:
    1. Extract frames at 1 fps
    2. Run SLAM on frames folder
    3. Trigger MapControl command 5 every ~1s to save frame+pose
    4. Collect saved frames/poses from procframe dir
    """
    session = _load_session(req.session_id)
    if session["status"] not in (SessionStatus.PROCESSING, SessionStatus.TRIMMING):
        raise HTTPException(400, detail=f"Session not in processable state: {session['status']}")

    trimmed_path = Path(session.get("trimmed_video_path") or session["video_path"])
    if not trimmed_path.exists():
        raise HTTPException(404, detail="Trimmed video not found")

    project_id = req.project_id or session["project_id"]
    frames_dir = PROJECTS_DIR / project_id / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    # Clear existing frames
    for f in frames_dir.glob("*.jpg"):
        f.unlink()

    session["frames_dir"] = str(frames_dir)
    session["project_id"] = project_id
    session["status"] = SessionStatus.PROCESSING
    _save_session(session)

    # Step 1: Extract frames at 1 fps
    try:
        cmd = [
            "ffmpeg", "-y", "-i", str(trimmed_path),
            "-vf", "fps=1",
            str(frames_dir / "%04d.jpg")
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"Frame extraction failed: {result.stderr}")
    except Exception as e:
        session["status"] = SessionStatus.ERROR
        _save_session(session)
        raise HTTPException(500, detail=f"Frame extraction failed: {e}")

    # Count extracted frames
    frame_files = sorted(frames_dir.glob("*.jpg"))
    frame_count = len(frame_files)
    if frame_count == 0:
        session["status"] = SessionStatus.ERROR
        _save_session(session)
        raise HTTPException(500, detail="No frames extracted")

    # Estimate video duration from frame count (1 fps)
    duration = frame_count

    # Step 2: Run SLAM on frames folder
    try:
        # Update YAML for this project
        yaml_path = "/home/orb/Database/real.yaml"
        save_path = f"/home/orb/Database/projects/{project_id}/calibrations/map"
        # Reuse update_yaml from process_manager
        from .process_manager import update_yaml
        update_yaml(yaml_path, save_path)

        # Start publisher_folder
        publisher_name = "publisher_folder"
        await manager.start(publisher_name, extra={"path": str(frames_dir)})
        await asyncio.sleep(2)

        # Start SLAM
        await manager.start("slam")

        # Step 3: Trigger MapControl command 5 every ~1s during SLAM run
        # We'll run for 'duration' seconds, triggering save every second
        for i in range(duration):
            await asyncio.sleep(1)
            # Call MapControl command 5 via ROS2 service
            try:
                await ros2_run([
                    "service", "call", "/orb_slam3/map_control",
                    "orb_slam3_ros2_wrapper/srv/MapControl",
                    "{command: 5, filepath: ''}"
                ], timeout=2)
            except Exception as e:
                print(f"[calibration] MapControl cmd 5 failed at frame {i}: {e}")

        # Stop publisher and SLAM
        await manager.stop(publisher_name)
        await asyncio.sleep(2)
        await manager.stop("slam")

    except Exception as e:
        session["status"] = SessionStatus.ERROR
        _save_session(session)
        raise HTTPException(500, detail=f"SLAM processing failed: {e}")

    # Step 4: Collect saved frames/poses from procframe dir
    frame_pose_data = []
    for frame_file in sorted(PROCFRAME_DIR.glob("frame_*.jpg")):
        txt_file = frame_file.with_suffix(".txt")
        pose = None
        if txt_file.exists():
            try:
                with open(txt_file, "r") as f:
                    line = f.read().strip()
                    if line:
                        x, y, z = map(float, line.split())
                        pose = {"x": x, "y": y, "z": z}
            except Exception:
                pass
        frame_pose_data.append({
            "frame": frame_file.name,
            "pose_file": txt_file.name if txt_file.exists() else None,
            "pose": pose,
            "timestamp": int(frame_file.stat().st_mtime)
        })

    # Copy procframe files to session folder for persistence
    session_procframe = SESSIONS_DIR / f"{req.session_id}_procframe"
    session_procframe.mkdir(exist_ok=True)
    for f in PROCFRAME_DIR.glob("frame_*"):
        import shutil
        shutil.copy2(f, session_procframe / f.name)

    session["frame_pose_data"] = frame_pose_data
    session["status"] = SessionStatus.CORRELATING
    _save_session(session)

    return {
        "success": True,
        "session": session,
        "frames_extracted": frame_count,
        "poses_saved": len([f for f in frame_pose_data if f["pose"]]),
        "procframe_dir": str(session_procframe)
    }


# ========== FRAME LISTING ==========

@router.get("/api/v1/calibration/session/{session_id}/frames")
async def list_frames(session_id: str):
    """List frames with poses from session's procframe data."""
    session = _load_session(session_id)
    return {
        "success": True,
        "frames": session.get("frame_pose_data", []),
        "project_id": session.get("project_id")
    }


# ========== 5-POINT CORRELATION ==========

@router.post("/api/v1/calibration/session/{session_id}/correlate")
async def correlate_points(session_id: str, req: CorrelateReq):
    """
    Compute similarity transform from 5 ORB points + 5 GPS points.
    Uses make_transform.py logic.
    """
    session = _load_session(session_id)
    if session["status"] not in (SessionStatus.CORRELATING, SessionStatus.PROCESSING):
        raise HTTPException(400, detail=f"Session not in correlatable state: {session['status']}")

    if len(req.points) != 5:
        raise HTTPException(400, detail="Exactly 5 correlation points required")

    # Prepare data for make_transform
    gps_points = []
    orb_points = []
    for p in req.points:
        gps_points.append([p.lat, p.lon, p.alt])
        # We need the ORB pose for this frame_idx
        frame_data = session["frame_pose_data"]
        if p.frame_idx >= len(frame_data):
            raise HTTPException(400, detail=f"frame_idx {p.frame_idx} out of range")
        pose = frame_data[p.frame_idx].get("pose")
        if not pose:
            raise HTTPException(400, detail=f"No pose for frame {p.frame_idx}")
        orb_points.append([pose["x"], pose["y"], pose["z"]])

    import numpy as np
    gps_points = np.array(gps_points)
    orb_points = np.array(orb_points)

    # Use first point as reference
    lat0, lon0, alt0 = req.points[0].lat, req.points[0].lon, req.points[0].alt

    # Compute similarity transform (copied from make_transform.py)
    def geodetic_to_enu(points, lat0, lon0, alt0):
        a = 6378137.0
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
            t = np.array([
                [-np.sin(lon_rad), np.cos(lon_rad), 0],
                [-np.sin(lat_rad) * np.cos(lon_rad), -np.sin(lat_rad) * np.sin(lon_rad), np.cos(lat_rad)],
                [np.cos(lat_rad) * np.cos(lon_rad), np.cos(lat_rad) * np.sin(lon_rad), np.sin(lat_rad)]
            ])
            enu = t @ dx
            enu_points.append(enu)
        return np.array(enu_points)

    enu_points = geodetic_to_enu(gps_points, lat0, lon0, alt0)
    orb = orb_points

    centroid_orb = np.mean(orb, axis=0)
    centroid_enu = np.mean(enu_points, axis=0)
    orb_centered = orb - centroid_orb
    enu_centered = enu_points - centroid_enu

    scale = np.sqrt(np.sum(enu_centered ** 2)) / np.sqrt(np.sum(orb_centered ** 2))
    H = orb_centered.T @ enu_centered
    U, S, Vt = np.linalg.svd(H)
    R_matrix = Vt.T @ U.T
    if np.linalg.det(R_matrix) < 0:
        Vt[-1, :] *= -1
        R_matrix = Vt.T @ U.T
    translation = centroid_enu - scale * R_matrix @ centroid_orb

    transform = {
        "scale": float(scale),
        "rotation": R_matrix.tolist(),
        "translation": translation.tolist(),
        "lat0": float(lat0),
        "lon0": float(lon0),
        "alt0": float(alt0)
    }

    session["transform"] = transform
    session["correlation_points"] = [p.model_dump() for p in req.points]
    session["status"] = SessionStatus.DONE
    _save_session(session)

    return {"success": True, "transform": transform, "session": session}


# ========== FINALIZE CALIBRATION ==========

@router.post("/api/v1/calibration/session/{session_id}/finalize")
async def finalize_calibration(session_id: str):
    """Generate calib.gpc from transform and save to both locations."""
    session = _load_session(session_id)
    if session["status"] != SessionStatus.DONE:
        raise HTTPException(400, detail=f"Session not ready for finalize: {session['status']}")

    transform = session.get("transform")
    if not transform:
        raise HTTPException(400, detail="No transform computed")

    # Generate calib.gpc content
    # Format: proj string + 5 lines of "X Y Z Lon Lat Alt"
    proj_line = "+proj=utm +zone=38 +datum=WGS84 +units=m +no_defs"
    lines = [proj_line]

    # Use correlation points to generate the 5 lines in calib.gpc format
    # calib.gpc expects: X Y Z Lon Lat Alt (ORB local coords + GPS coords)
    for p in session["correlation_points"]:
        # Get ORB pose for this frame
        frame_data = session["frame_pose_data"]
        pose = frame_data[p["frame_idx"]].get("pose") if p["frame_idx"] < len(frame_data) else None
        if pose:
            lines.append(f"{pose['x']:.6f} {pose['y']:.6f} {pose['z']:.6f} {p['lon']:.8f} {p['lat']:.8f} {p['alt']:.3f}")

    calib_content = "\n".join(lines) + "\n"

    # Save to system default location
    system_calib = Path("/opt/main/Trajectory/TerraSLAM_relay/Serial/calib.gpc")
    system_calib.parent.mkdir(parents=True, exist_ok=True)
    with open(system_calib, "w", encoding="utf-8") as f:
        f.write(calib_content)

    # Save to project-specific location
    project_id = session["project_id"]
    project_calib = PROJECTS_DIR / project_id / "calibrations" / "calib.gpc"
    project_calib.parent.mkdir(parents=True, exist_ok=True)
    with open(project_calib, "w", encoding="utf-8") as f:
        f.write(calib_content)

    session["calib_gpc_path"] = str(project_calib)
    session["status"] = SessionStatus.DONE
    _save_session(session)

    return {
        "success": True,
        "system_calib": str(system_calib),
        "project_calib": str(project_calib),
        "content": calib_content,
        "session": session
    }


@router.get("/api/v1/calibration/session/{session_id}/video")
async def get_session_video(session_id: str):
    """Serve raw video file for a calibration session."""
    session = _load_session(session_id)
    video_path = Path(session.get("video_path", ""))
    if not video_path.exists():
        raise HTTPException(404, detail="Video not found")
    return FileResponse(video_path, media_type="video/mp4")


@router.post("/api/v1/calibration/video/{session_id}/upload")
async def upload_session_video(session_id: str, file: UploadFile = File(...)):
    """Upload a complete video file for a calibration session (simulation mode)."""
    session = _load_session(session_id)
    if session["status"] != SessionStatus.RECORDING:
        raise HTTPException(400, detail=f"Session not in recording state: {session['status']}")

    video_path = Path(session["video_path"])
    content = await file.read()
    with open(video_path, "wb") as f:
        f.write(content)

    session["status"] = SessionStatus.TRIMMING
    _save_session(session)

    return {"success": True, "bytes_written": len(content), "session": session}
