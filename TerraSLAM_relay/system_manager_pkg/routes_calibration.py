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
from typing import Dict, List, Optional

from fastapi import APIRouter, HTTPException, UploadFile, File, Form
from fastapi.responses import FileResponse
from pydantic import BaseModel

from .config import RunReq
from .process_manager import manager
from .ros_utils import ros2_run
from .routes_control import control_terraslam_component as _control_component, ComponentAction as _ComponentAction
from .routes_projects import read_project_metadata, write_project_metadata

router = APIRouter()

PROJECTS_DIR = Path(os.getenv("PROJECTS_DIR", "/opt/main/Trajectory/Database/projects"))


def _calib_dir(project_id: str) -> Path:
    """Per-project calibration session directory: projects/<id>/calib."""
    d = PROJECTS_DIR / project_id / "calib"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _procframe_dir(project_id: Optional[str]) -> Path:
    """Strict per-project procframe directory.

    Processed frames + poses MUST be saved under projects/<id>/procframe —
    never a shared global path. A project id is required; otherwise we
    refuse to proceed.
    """
    if not project_id:
        raise HTTPException(
            400,
            detail="project_id is required: procframe results must be saved under "
                   "projects/<id>/procframe",
        )
    return PROJECTS_DIR / project_id / "procframe"


# --- procframe saving gate ---
# The mono SLAM node writes processed frames into procframe ONLY while this flag
# file exists. We create it at the start of a calibration run and remove it at
# every exit path, so normal operation (recording / flight) never persists frames.
SAVE_FRAMES_FLAG = "/tmp/terraslam_save_frames"


def _enable_frame_saving(session_id: str) -> None:
    try:
        with open(SAVE_FRAMES_FLAG, "w") as fh:
            fh.write(f"calibration_session={session_id}\n")
        print(f"[frame-save] ENABLED ({SAVE_FRAMES_FLAG})", flush=True)
    except OSError as exc:
        print(f"[frame-save] WARNING cannot create flag: {exc}", flush=True)


def _disable_frame_saving() -> None:
    try:
        if os.path.exists(SAVE_FRAMES_FLAG):
            os.remove(SAVE_FRAMES_FLAG)
        print(f"[frame-save] DISABLED ({SAVE_FRAMES_FLAG})", flush=True)
    except OSError as exc:
        print(f"[frame-save] WARNING cannot remove flag: {exc}", flush=True)


# --- In-memory calibration progress tracker ---
_calib_progress: Dict[str, dict] = {}

def _update_progress(session_id: str, **kwargs):
    p = _calib_progress.setdefault(session_id, {})
    p.update(kwargs)
    p["updated_at"] = time.time()


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


# Calibration session is always stored as a single fixed file per project:
# projects/<id>/calibrations/1.json — no random id in the filename.
SESSION_FILENAME = "1.json"


def _session_dir(project_id: str) -> Path:
    """Per-project calibration session directory: projects/<id>/calibrations."""
    d = PROJECTS_DIR / project_id / "calibrations"
    d.mkdir(parents=True, exist_ok=True)
    return d


def _session_path(session_id: str, project_id: Optional[str] = None) -> Path:
    """Resolve the session JSON path.

    Sessions live under projects/<id>/calibrations/1.json. If project_id
    is known it is used directly; otherwise every project's calibrations dir
    is searched. Because ALL projects share the fixed filename 1.json, we must
    match by the session id stored INSIDE the file — otherwise we'd return a
    different project's session (e.g. one already in "correlating") and every
    operation on the real session would hit the wrong file.
    """
    if project_id:
        return _session_dir(project_id) / SESSION_FILENAME
    # Prefer the project whose 1.json actually holds this session id.
    for proj in PROJECTS_DIR.glob("*"):
        cand = proj / "calibrations" / SESSION_FILENAME
        if cand.exists():
            try:
                with open(cand, "r", encoding="utf-8-sig") as f:
                    data = json.load(f)
                if data.get("id") == session_id:
                    return cand
            except Exception:
                continue
    # No project owns this session id: do NOT fall back to an arbitrary
    # project's 1.json, otherwise operations (stop/resume) silently hit the
    # wrong project and stop/return a foreign recording session.
    raise HTTPException(404, detail=f"Session {session_id} not found")


def _load_session(session_id: str, project_id: Optional[str] = None) -> dict:
    path = _session_path(session_id, project_id)
    if not path.exists():
        raise HTTPException(404, detail=f"Session {session_id} not found")
    try:
        # utf-8-sig съедает BOM, если файл записан редактором на Windows.
        with open(path, "r", encoding="utf-8-sig") as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        raise HTTPException(400, detail=f"Corrupt session file {path}: {e}")
    if not isinstance(data, dict):
        raise HTTPException(400, detail=f"Session file {path} is not a JSON object")
    return data


def _save_session(session: dict):
    path = _session_path(session.get("id", ""), session.get("project_id"))
    path.parent.mkdir(parents=True, exist_ok=True)
    # Атомарная запись: пишем во временный файл и переименовываем, чтобы
    # прерывание процесса не оставляло битый/незаконченный JSON (иначе
    # следующий _load_session упадёт с 500 Internal Server Error).
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(session, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def _new_session_id() -> str:
    return f"calib_{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"


# ========== SESSION CRUD ==========

@router.post("/api/v1/calibration/video/start")
async def start_recording(req: Optional[Dict] = None):
    """Create a new calibration recording session."""
    project_id = (req or {}).get("project_id")
    # Только при явном старте записи (clear_existing_frames=true) стираем
    # старые кадры из frames и запускаем publisher. При пропуске экрана
    # (clear_existing_frames=false / не передан) НИЧЕГО не удаляем и не
    # запускаем publisher — чтобы не было перекрестия кадров.
    clear_existing_frames = (req or {}).get("clear_existing_frames", False)
    session_id = _new_session_id()
    if not project_id:
        raise HTTPException(400, detail="project_id is required to start a calibration session")

    # При явном старте записи сначала (clear_existing_frames=true) стираем
    # предыдущий файл сессии калибровки projects/<id>/calibrations/1.json,
    # чтобы не осталось данных прошлой калибровки.
    if clear_existing_frames:
        old_session = _session_dir(project_id) / SESSION_FILENAME
        try:
            if old_session.exists():
                old_session.unlink()
                print(f"[start_recording] removed stale session file: {old_session}", flush=True)
        except OSError as e:
            print(f"[start_recording] WARNING cannot remove {old_session}: {e}", flush=True)

    video_path = _calib_dir(project_id) / f"{session_id}_raw.mp4"

    frames_dir_value = str(PROJECTS_DIR / project_id / "frames")

    session = {
        "id": session_id,
        "created_at": int(time.time()),
        "status": SessionStatus.RECORDING,
        "video_path": str(video_path),
        "trimmed_video_path": None,
        "trim_segments": [],
        "project_id": project_id or session_id,
        "frames_dir": frames_dir_value,
        "procframe_dir": str(_procframe_dir(project_id)),
        "frame_pose_data": [],
        "correlation_points": [],
        "transform": None,
        "calib_gpc_path": None,
    }
    _save_session(session)

    # Очищаем папку frames проекта от предыдущей (прерванной) записи,
    # чтобы в ней остались только кадры текущей сессии. При пропуске
    # записи (clear_existing_frames=False) кадры сохраняем.
    if frames_dir_value and clear_existing_frames:
        fdir = Path(frames_dir_value)
        try:
            fdir.mkdir(parents=True, exist_ok=True)
            for old in fdir.glob("*.jpg"):
                old.unlink()
            for old in fdir.glob("*.txt"):
                old.unlink()
        except Exception as e:
            print(f"[start_recording] warning clearing frames dir: {e}")

    # Автономно стартуем publisher_realsense с папкой frames проекта,
    # чтобы кадры писались на диск даже если фронт не дёрнул этот эндпоинт.
    # Это основной путь записи для проектов типа «камера». При пропуске
    # записи publisher не запускаем (кадры уже есть).
        if project_id and clear_existing_frames:
            try:
                await manager.start("publisher_realsense", extra={"frames_dir": frames_dir_value})
                print(f"[start_recording] publisher_realsense started for frames_dir={frames_dir_value}")
            except Exception as e:
                print(f"[start_recording] WARNING: failed to start publisher_realsense: {e}")

    return {"success": True, "session": session}


@router.post("/api/v1/calibration/video/start-from-project/{project_id}")
async def start_recording_from_project(project_id: str):
    """Create a new calibration recording session from an existing project.

    Симуляция обрабатывается покадрово — так же, как режим камеры. Кадры уже
    нарезаны из видео проекта при загрузке видео и лежат в projects/<id>/frames,
    поэтому здесь НИЧЕГО не извлекаем и НЕ создаём копию видео/папку calib.
    Просто создаём сессию, указывающую на готовую папку frames — фронт и так
    заберёт кадры оттуда через /api/projects/:id/frames.
    """
    project_path = PROJECTS_DIR / project_id
    if not project_path.exists():
        raise HTTPException(404, detail=f"Project {project_id} not found")

    metadata_path = project_path / "metadata.json"
    if not metadata_path.exists():
        raise HTTPException(404, detail="Project metadata not found")

    frames_dir = project_path / "frames"
    if not frames_dir.exists() or not any(frames_dir.glob("*.jpg")):
        raise HTTPException(
            400,
            detail="Project has no extracted frames in frames/ — upload the project video first",
        )

    session_id = _new_session_id()

    session = {
        "id": session_id,
        "created_at": int(time.time()),
        "status": SessionStatus.TRIMMING,
        "video_path": None,
        "trimmed_video_path": None,
        "trim_segments": [],
        "project_id": project_id,
        "frames_dir": str(frames_dir),
        "procframe_dir": str(_procframe_dir(project_id)),
        "frame_pose_data": [],
        "correlation_points": [],
        "transform": None,
        "calib_gpc_path": None,
    }

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
    # Idempotent: if a previous stop already moved the session past recording
    # (e.g. the page was refreshed after Stop), just return the current state
    # instead of failing with 400.
    if session["status"] == SessionStatus.RECORDING:
        session["status"] = SessionStatus.TRIMMING
        session["recording_stopped_at"] = int(time.time())
        _save_session(session)

    # Always attempt to stop the publisher so a stale capture process is
    # terminated even on a repeated stop call.
    project_id = session.get("project_id")

    # Останавливаем publisher_realsense, чтобы он перестал дописывать кадры
    # в папку frames проекта после завершения записи.
    project_id = session.get("project_id")
    if project_id:
        try:
            await manager.stop("publisher_realsense")
            print(f"[stop_recording] publisher_realsense stopped for project {project_id}")
        except Exception as e:
            print(f"[stop_recording] WARNING: failed to stop publisher_realsense: {e}")

    return {"success": True, "session": session}


@router.get("/api/v1/calibration/sessions")
async def list_sessions():
    """List all calibration sessions."""
    sessions = []
    for proj in PROJECTS_DIR.glob("*"):
        path = proj / "calibrations" / SESSION_FILENAME
        if not path.exists():
            continue
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


@router.get("/api/v1/calibration/recording-status/{project_id}")
async def get_recording_status(project_id: str):
    """Report the live recording state for a project.

    The frontend uses this (e.g. after a page refresh) to discover whether a
    calibration recording is still in progress, instead of trusting only the
    in-memory UI state. We return the session status plus an authoritative
    check of whether the RealSense publisher process is actually running, so a
    session stuck in "recording" (e.g. the page was closed without pressing
    Stop) is reported as NOT recording when the capture process is gone.
    """

    path = PROJECTS_DIR / project_id / "calibrations" / SESSION_FILENAME

    if not path.exists():
        return {
            "recording": False,
            "status": None,
            "session_id": None,
            "elapsed": 0,
            "publisher_running": False,
            "server_time": int(time.time()),
        }

    try:
        with open(path, "r", encoding="utf-8-sig") as f:
            session = json.load(f)
    except Exception:
        return {
            "recording": False,
            "status": None,
            "session_id": None,
            "elapsed": 0,
            "publisher_running": False,
            "server_time": int(time.time()),
        }

    status = session.get("status")
    session_id = session.get("id")

    publisher_running = _is_realsense_publisher_alive(project_id)

    is_recording = status == SessionStatus.RECORDING and publisher_running

    # Self-heal: a session that is marked "recording" but has no live capture
    # process (e.g. the manager crashed, or the page was closed without Stop)
    # is stale. Flip it to "trimming" so the UI stops claiming a recording and
    # the next status poll is consistent. Only the project that owns this
    # session file is touched.
    if status == SessionStatus.RECORDING and not publisher_running:
        try:
            session["status"] = SessionStatus.TRIMMING
            session["recording_stopped_at"] = int(time.time())
            _save_session(session)
            print(
                f"[recording-status] healed stale 'recording' session "
                f"{session_id} (no live publisher) -> trimming",
                flush=True,
            )
            status = session.get("status")
        except Exception as exc:
            print(f"[recording-status] WARNING heal failed: {exc}", flush=True)

    # Elapsed recording time (seconds) as seen by the server clock.
    started_at = session.get("created_at") or session.get("recording_started_at")
    elapsed = 0
    if started_at and status == SessionStatus.RECORDING:
        elapsed = max(0, int(time.time()) - int(started_at))

    return {
        "recording": is_recording,
        "status": status,
        "session_id": session_id,
        "elapsed": elapsed,
        "publisher_running": publisher_running,
        "server_time": int(time.time()),
    }


def _is_realsense_publisher_alive(project_id: Optional[str] = None) -> bool:
    """Authoritative check of whether the RealSense capture is actually running.

    manager._procs is in-memory only and is wiped on every backend restart, so a
    session stuck in "recording" after the manager died would be reported as
    still recording even though no capture process exists. To stay correct after
    a restart we also probe the OS process table directly (the publisher is
    started with start_new_session=True, so it survives a manager crash as an
    orphan and keeps writing frames).

    During calibration recording the running process is publisher_realsense,
    which receives a project-specific --frames-dir argument. We therefore
    additionally match the process command line against this project's frames
    directory, so a publisher recording a *different* project is not mistaken
    for the current one.
    """
    info = manager._procs.get("publisher_realsense")
    proc = info.get("proc") if info else None
    if proc is not None and proc.returncode is None:
        return True

    try:
        if project_id:
            frames_dir = str(PROJECTS_DIR / project_id / "frames")
            pattern = f"frames-dir[ =]{frames_dir}"
            out = subprocess.run(
                ["pgrep", "-f", pattern],
                capture_output=True, text=True, timeout=3,
            )
            if out.returncode == 0 and out.stdout.strip():
                return True
        out = subprocess.run(
            ["pgrep", "-f", "publisher_realsense"],
            capture_output=True, text=True, timeout=3,
        )
        if out.returncode == 0 and out.stdout.strip():
            return True
    except Exception:
        pass
    return False


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

    trimmed_path = _calib_dir(session["project_id"]) / f"{session_id}_trimmed.mp4"

    # Build ffmpeg filter for multiple segments
    # Using concat demuxer with segment list
    segments = req.segments
    if not segments:
        raise HTTPException(400, detail="No trim segments provided")

    # Create a temp file list for concat
    list_path = _calib_dir(session["project_id"]) / f"{session_id}_segments.txt"
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

async def _wait_slam_alive(timeout: float = 15.0) -> bool:
    """Дождаться, пока процесс ORB-SLAM3 поднимется (не упал сразу после старта).

    Возвращает True, если процесс жив (returncode is None) в пределах timeout.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        info = manager._procs.get("slam", {})
        proc = info.get("proc")
        if proc is not None and proc.returncode is None:
            return True
        await asyncio.sleep(0.5)
    return False


def _slam_is_up() -> bool:
    """SLAM считается запущенным, если им управляет этот менеджер, либо если
    внешний SLAM (supervisor/сервис) пишет свежий файл статуса
    (/tmp/terraslam_slam_status)."""
    info = manager._procs.get("slam", {})
    proc = info.get("proc")
    if proc is not None and proc.returncode is None:
        return True
    status_file = "/tmp/terraslam_slam_status"
    try:
        if os.path.exists(status_file) and (time.time() - os.path.getmtime(status_file)) <= 10.0:
            with open(status_file) as f:
                data = json.loads(f.read().strip())
            if data.get("slam_state", -1) != -1 or data.get("initialized"):
                return True
    except Exception:
        pass
    return False


async def _run_calibration_process(session_id: str, session: dict, project_id: str,
                                    frames_dir: Path, trimmed_path: Path):
    """Background task: extract frames → run SLAM → collect poses. Updates _calib_progress."""
    import shutil
    started_at = time.time()

    def upd(**kw):
        _update_progress(session_id, started_at=started_at,
                         elapsed=round(time.time() - started_at, 1), **kw)

    print(f"[calib:{session_id}] START process_calibration "
          f"(project={project_id}, frames_dir={frames_dir}, trimmed_path={trimmed_path})", flush=True)

    # Step 1: Extract frames (or reuse frames already captured by camera)
    upd(step="extracting_frames", step_label="Извлечение кадров",
        frames_total=0, frames_done=0, slam_running=False, slam_crashed=False, error=None)

    existing_frames = sorted(frames_dir.glob("*.jpg"))
    have_video = trimmed_path is not None and trimmed_path.exists() and trimmed_path.stat().st_size > 0

    print(f"[calib:{session_id}] step1: have_video={have_video} "
          f"existing_frames={len(existing_frames)}", flush=True)

    if have_video:
        try:
            cmd = [
                "ffmpeg", "-y", "-i", str(trimmed_path),
                "-vf", "fps=30",
                str(frames_dir / "%04d.jpg")
            ]
            print(f"[calib:{session_id}] step1: ffmpeg extract frames: {' '.join(cmd)}", flush=True)
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                raise RuntimeError(f"Frame extraction failed: {result.stderr[-500:]}")
            print(f"[calib:{session_id}] step1: ffmpeg done (rc={result.returncode})", flush=True)
        except Exception as e:
            session["status"] = SessionStatus.ERROR
            _save_session(session)
            upd(step="error", step_label="Ошибка извлечения кадров", error=str(e))
            print(f"[calib:{session_id}] step1 ERROR: {e}", flush=True)
            return
    elif existing_frames:
        # Камера (RealSense): кадры уже на диске — пропускаем извлечение
        upd(step="starting_slam", step_label="Кадры получены с камеры",
            frames_total=len(existing_frames), frames_done=0,
            slam_running=False, slam_crashed=False, error=None)
    else:
        session["status"] = SessionStatus.ERROR
        _save_session(session)
        upd(step="error", step_label="Ошибка", error="Нет видео и нет кадров для обработки")
        print(f"[calib:{session_id}] step1 ERROR: no video and no frames", flush=True)
        return

    frame_files = sorted(frames_dir.glob("*.jpg"))
    frame_count = len(frame_files)
    if frame_count == 0:
        session["status"] = SessionStatus.ERROR
        _save_session(session)
        upd(step="error", step_label="Ошибка", error="Кадры не извлечены (ffmpeg вернул 0 файлов)")
        print(f"[calib:{session_id}] step1 ERROR: 0 frames extracted", flush=True)
        return

    print(f"[calib:{session_id}] step1: frame_count={frame_count}", flush=True)
    upd(step="starting_slam", step_label="Запуск SLAM", frames_total=frame_count, frames_done=0)

    # Per-project procframe directory — image_publish saves frame.jpg+pose.txt here.
    # Must match the path the user inspects: Database/projects/<id>/procframe.
    procframe_dir = PROJECTS_DIR / project_id / "procframe"
    procframe_dir.mkdir(parents=True, exist_ok=True)
    print(f"[calib:{session_id}] procframe_dir = {procframe_dir}", flush=True)

    # Step 2: Сначала запускаем SLAM, и только после его успешного старта —
    # image publisher. Кадры издателя нужны SLAM для инициализации трекинга,
    # поэтому издатель стартует строго после поднятия процесса SLAM.
    publisher_name = "publisher_folder"
    try:
        yaml_path = "/opt/main/Trajectory/Database/real.yaml"
        # ORB-SLAM3 SaveAtlas() writes to "./<SaveAtlasToFile>.osa" relative to
        # the SLAM process CWD (/opt/main/Trajectory). An absolute path there
        # would be concatenated into "./<absolute>.osa" and never created, so we
        # must pass a path RELATIVE to /opt/main/Trajectory (no leading slash,
        # no ".osa" suffix — ORB-SLAM3 adds it).
        save_path = f"Database/projects/{project_id}/calibrations/map"
        from .process_manager import update_yaml

        # Удаляем старый .osa перед запуском SLAM, чтобы при save_map не
        # возникло конфликта с предыдущей картой проекта. ORB-SLAM не всегда
        # корректно перезаписывает существующий файл, поэтому стираем явно.
        # save_path относительный от CWD SLAM (/opt/main/Trajectory), резолвим
        # его к абсолютному, чтобы os.remove нашёл файл из процесса бэкенда.
        osa_path = f"/opt/main/Trajectory/Database/projects/{project_id}/calibrations/map.osa"
        try:
            if os.path.exists(osa_path):
                os.remove(osa_path)
                print(f"[calib:{session_id}] step2: removed stale map.osa: {osa_path}", flush=True)
        except OSError as e:
            print(f"[calib:{session_id}] step2: WARNING cannot remove {osa_path}: {e}", flush=True)

        # В yaml НЕ должно быть загрузки карты (LoadAtlasFromFile) — только
        # сохранение (SaveAtlasToFile), чтобы SLAM строил карту заново, а не
        # подгружал старую .osa проекта.
        print(f"[calib:{session_id}] step2: update_yaml({yaml_path} -> {save_path})", flush=True)
        update_yaml(yaml_path, save_path)

        # Проверяем, включен ли SLAM.
        # - если им управляет этот менеджер и он жив — выключаем и перезапускаем
        #   (требование: «если включён — выключаем и перезапускаем»);
        # - если SLAM уже запущен внешне (supervisor/сервис, пишет файл статуса)
        #   — оставляем как есть и НЕ поднимаем второй экземпляр (иначе конфликт
        #   ROS-узла и процесс сразу падает → ложная ошибка);
        # - иначе — запускаем.
        managed = manager._procs.get("slam", {}).get("proc")
        if managed is not None and managed.returncode is None:
            print(f"[calib:{session_id}] step2: SLAM managed & running — restarting", flush=True)
            upd(step="starting_slam", step_label="Перезапуск SLAM",
                frames_total=frame_count, frames_done=0,
                slam_running=False, slam_crashed=False)
            await manager.restart("slam")
        elif _slam_is_up():
            print(f"[calib:{session_id}] step2: SLAM already running (external) — reusing", flush=True)
            upd(step="starting_slam", step_label="SLAM уже запущен",
                frames_total=frame_count, frames_done=0,
                slam_running=True, slam_crashed=False)
        else:
            print(f"[calib:{session_id}] step2: SLAM not running — starting", flush=True)
            upd(step="starting_slam", step_label="Запуск SLAM",
                frames_total=frame_count, frames_done=0,
                slam_running=False, slam_crashed=False)
            await manager.start("slam")

        # Лучшее усилие: дождаться подъёма процесса. НЕ фатально — если SLAM
        # медленно инициализируется или управляется извне, продолжаем; реальный
        # сбой SLAM будет пойман проверкой здоровья в цикле обработки.
        slam_alive = await _wait_slam_alive(timeout=15.0)
        if not slam_alive:
            print(f"[calib:{session_id}] step2 WARN: SLAM not confirmed alive after start "
                  f"— proceeding (loop health-check will catch a real crash)", flush=True)

        # Только после успешного старта SLAM запускаем image publisher.
        print(f"[calib:{session_id}] step2: manager.start({publisher_name}, "
              f"path={frames_dir}, procframe_dir={procframe_dir})", flush=True)
        upd(step="slam_ready", step_label="SLAM запущен, запуск издателя кадров",
            frames_total=frame_count, frames_done=0,
            slam_running=True, slam_crashed=False)
        await manager.start(publisher_name, extra={
            "path": str(frames_dir),
            "once": True,
            "procframe_dir": str(procframe_dir),
        })

        # Новый ORB-SLAM3 врапер не имеет старого MapControl command 5, который
        # писал кадр+позу в procframe. Вместо него отдельный узел
        # procframe_capture подписывается на /orb_slam3/robot_pose_slam и
        # камеру и пишет пары в procframe, пока поднят флаг сохранения кадров.
        try:
            await manager.start("procframe_capture", extra={
                "procframe_dir": str(procframe_dir),
            })
            print(f"[calib:{session_id}] step2: procframe_capture started for {procframe_dir}", flush=True)
        except Exception as e:
            print(f"[calib:{session_id}] step2 WARNING: failed to start procframe_capture: {e}", flush=True)

        _enable_frame_saving(session_id)
        upd(step="publisher_running", step_label="Издатель кадров публикует...",
            frames_total=frame_count, frames_done=0,
            slam_running=True, slam_crashed=False)
    except Exception as e:
        session["status"] = SessionStatus.ERROR
        _save_session(session)
        _disable_frame_saving()
        upd(step="error", step_label="Ошибка запуска SLAM/издателя", error=str(e),
            slam_running=False, slam_crashed=False)
        print(f"[calib:{session_id}] step2 ERROR starting slam/publisher: {e}", flush=True)
        return

    # Step 3: Прогон кадров. Статус обработки (= число кадров, которые
    # ИЗДАТЕЛЬ УЖЕ ОТПРАВИЛ) считается по реальному времени работы издателя,
    # а НЕ по счётчику итераций цикла. Сам цикл замедляется блокирующим
    # вызовом ros2 (MapControl занимает ~0.5с), поэтому привязка прогресса к
    # i+1 давала ~2 кадра/сек вместо реальных ~30 кадров/сек, которые шлёт
    # издатель. Теперь frames_done растёт со скоростью издателя (по умолчанию
    # 30 fps, либо с реальным fps издателя).
    slam_crashed = False
    publisher_fps = 30.0  # image_publish по умолчанию 30 fps (реальный fps издателя)
    publisher_start = time.time()
    frame_interval = 1.0 / publisher_fps

    while True:
        # Здоровье издателя
        pub_info = manager._procs.get(publisher_name, {})
        pub_proc = pub_info.get("proc")
        if pub_proc is not None and pub_proc.returncode is not None:
            sent = min(frame_count, int((time.time() - publisher_start) * publisher_fps))
            upd(step="error", step_label="Издатель кадров аварийно завершился",
                frames_done=sent, slam_running=False, slam_crashed=False,
                error=f"Издатель кадров завершился с кодом {pub_proc.returncode}")
            print(f"[calib:{session_id}] step3: publisher crashed "
                  f"(rc={pub_proc.returncode})", flush=True)
            break

        # Здоровье SLAM
        slam_info = manager._procs.get("slam", {})
        slam_proc = slam_info.get("proc")
        if slam_proc is not None and slam_proc.returncode is not None:
            sent = min(frame_count, int((time.time() - publisher_start) * publisher_fps))
            slam_crashed = True
            upd(step="error", step_label="SLAM аварийно завершился",
                frames_done=sent, slam_running=False, slam_crashed=True,
                error=f"SLAM завершился с кодом {slam_proc.returncode}")
            print(f"[calib:{session_id}] step3: SLAM crashed "
                  f"(rc={slam_proc.returncode})", flush=True)
            break

        # Число кадров, которые издатель УЖЕ отправил (по реальному времени).
        sent = min(frame_count, int((time.time() - publisher_start) * publisher_fps))

        # Кадр+поза в procframe теперь пишутся узлом procframe_capture
        # (подписка на /orb_slam3/robot_pose_slam + камеру) пока поднят флаг
        # сохранения кадров (_enable_frame_saving). Явный MapControl command 5
        # из старого врапера здесь больше не вызывается.

        if (sent + 1) % 30 == 0 or sent == 0:
            print(f"[calib:{session_id}] step3: frame {sent}/{frame_count} sent", flush=True)
        upd(step="processing_frames", step_label="SLAM обрабатывает кадры",
            frames_done=sent, slam_running=True, slam_crashed=False)

        if sent >= frame_count:
            break
        await asyncio.sleep(frame_interval)

    # Сохраняем карту ПОКА SLAM ещё запущен: сервис /orb_slam3/save_map
    # (SetBool -> saveAtlas) недоступен после остановки процесса. Путь
    # SaveAtlasToFile уже прописан в real.yaml относительно CWD SLAM
    # (Database/projects/<id>/calibrations/map) на этапе старта (step2),
    # поэтому ORB-SLAM3 запишет <cwd>/Database/projects/<id>/calibrations/map.osa.
    try:
        out, err, rc = await asyncio.to_thread(
            ros2_run,
            ["service", "call", "/orb_slam3/save_map", "std_srvs/srv/SetBool", "{data: true}"],
            timeout=30,
        )
        print(f"[calib:{session_id}] save_map rc={rc} out={out} err={err}", flush=True)
    except Exception as e:
        print(f"[calib:{session_id}] WARNING save_map failed (non-fatal): {e}", flush=True)

    # Stop publisher, procframe capture and SLAM
    print(f"[calib:{session_id}] step3 done: stopping {publisher_name}, procframe_capture and slam", flush=True)
    _disable_frame_saving()
    await manager.stop(publisher_name)
    try:
        await manager.stop("procframe_capture")
    except Exception as e:
        print(f"[calib:{session_id}] step3 WARNING stopping procframe_capture: {e}", flush=True)
    await asyncio.sleep(2)
    await manager.stop("slam")

    if slam_crashed:
        session["status"] = SessionStatus.ERROR
        _save_session(session)
        print(f"[calib:{session_id}] FINISH with SLAM crash", flush=True)
        return

    upd(step="collecting_poses", step_label="Сбор поз",
        slam_running=False, slam_crashed=False)

    # Step 4: Collect poses from procframe dir.
    # image_publish saves each processed frame as <source_stem>.jpg + .txt,
    # so we glob every *.jpg here (NOT frame_*.jpg).
    frame_pose_data = []
    procframe_jpgs = sorted(procframe_dir.glob("*.jpg"))
    print(f"[calib:{session_id}] step4: found {len(procframe_jpgs)} procframe images "
          f"in {procframe_dir}", flush=True)
    for frame_file in procframe_jpgs:
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

    valid_frame_pose_data = []
    for entry in frame_pose_data:
        if entry["pose"] is None:
            frame_file = procframe_dir / entry["frame"]
            txt_file = frame_file.with_suffix(".txt")
            try:
                if frame_file.exists():
                    frame_file.unlink()
                if txt_file.exists():
                    txt_file.unlink()
                print(f"[calib:{session_id}] step4: removed frame without pose: {entry['frame']}", flush=True)
            except Exception as e:
                print(f"[calib:{session_id}] step4: warning removing {entry['frame']}: {e}", flush=True)
        else:
            valid_frame_pose_data.append(entry)

    poses_saved = len(valid_frame_pose_data)
    if poses_saved == 0:
        _disable_frame_saving()
        session["status"] = SessionStatus.ERROR
        session["frame_pose_data"] = []
        _save_session(session)
        upd(step="error", step_label="Ошибка обработки",
            frames_done=frame_count, slam_running=False, slam_crashed=False,
            error="Не найдено кадров с валидными позами. Карта не создана.")
        print(f"[calib:{session_id}] FINISH: no valid poses, status=ERROR", flush=True)
        return

    session["frame_pose_data"] = valid_frame_pose_data
    session["status"] = SessionStatus.CORRELATING
    _save_session(session)

    # Карта уже сохранена через /orb_slam3/save_map (см. конец step3, пока
    # SLAM ещё запущен — сервис недоступен после остановки процесса).

    print(f"[calib:{session_id}] FINISH: poses_saved={poses_saved}, "
          f"status={session['status']}, procframe_dir={procframe_dir}", flush=True)
    upd(step="done", step_label="Завершено",
        frames_done=frame_count, slam_running=False, slam_crashed=False,
        poses_saved=poses_saved, procframe_dir=str(procframe_dir))


@router.post("/api/v1/calibration/process")
async def process_calibration(req: ProcessReq):
    """
    Start background processing of trimmed video:
    1. Extract frames at 30 fps
    2. Run SLAM on frames folder
    3. Trigger MapControl command 5 every frame (~30 fps) to save frame+pose
    4. Collect saved frames/poses from procframe dir
    Returns immediately; poll /progress for status.
    """
    session = _load_session(req.session_id)
    status = session.get("status")
    if status not in (
        SessionStatus.RECORDING,
        SessionStatus.PROCESSING,
        SessionStatus.TRIMMING,
        SessionStatus.ERROR,
        SessionStatus.CORRELATING,
        SessionStatus.DONE,
    ):
        raise HTTPException(400, detail=f"Session not in processable state: {status}")

    project_id = req.project_id or session["project_id"]
    frames_dir = PROJECTS_DIR / project_id / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    # Камера (RealSense) и симуляция: кадры уже лежат в frames_dir, видеофайла
    # нет. Видео учитывается только если есть trimmed_video_path (режим
    # видео-обрезки). Иначе обрабатываем имеющиеся кадры проекта.
    trimmed_video = session.get("trimmed_video_path")
    if trimmed_video and Path(trimmed_video).exists() and Path(trimmed_video).stat().st_size > 0:
        trimmed_path: Optional[Path] = Path(trimmed_video)
    else:
        trimmed_path = None
    have_video = trimmed_path is not None
    existing_frames = sorted(frames_dir.glob("*.jpg"))

    if not have_video and not existing_frames:
        raise HTTPException(404, detail="Trimmed video not found and no captured frames")

    # Не удаляем уже отснятые кадры камеры — они и есть источник для SLAM
    if have_video:
        for f in frames_dir.glob("*.jpg"):
            f.unlink()

    session["frames_dir"] = str(frames_dir)
    session["project_id"] = project_id
    session["status"] = SessionStatus.PROCESSING
    _save_session(session)

    # Сразу при нажатии «Обработать» удаляем старый .osa проекта, чтобы при
    # последующем save_map в SLAM не было конфликта с предыдущей картой.
    calib_osa = PROJECTS_DIR / project_id / "calibrations" / "map.osa"
    try:
        if calib_osa.exists():
            calib_osa.unlink()
            print(f"[calib:{req.session_id}] removed stale map.osa on process start: {calib_osa}", flush=True)
    except OSError as e:
        print(f"[calib:{req.session_id}] WARNING cannot remove {calib_osa}: {e}", flush=True)

    # Reset progress and launch background task
    _calib_progress.pop(req.session_id, None)
    asyncio.create_task(
        _run_calibration_process(req.session_id, session, project_id, frames_dir, trimmed_path)
    )

    return {
        "success": True,
        "session_id": req.session_id,
        "message": "Обработка запущена в фоне. Опрашивайте /progress для статуса."
    }


@router.get("/api/v1/calibration/session/{session_id}/progress")
async def get_calibration_progress(session_id: str):
    """Return current progress of a calibration processing job."""
    progress = _calib_progress.get(session_id)
    if not progress:
        session = _load_session(session_id)  # raises 404 if not found
        return {"found": False, "session_status": session["status"]}
    return {"found": True, **progress}


# ========== FRAME LISTING ==========

@router.get("/api/v1/calibration/session/{session_id}/frames")
async def list_frames(session_id: str):
    """List frames with poses from session's procframe data."""
    session = _load_session(session_id)
    frames = [f for f in session.get("frame_pose_data", []) if f.get("pose")]
    return {
        "success": True,
        "frames": frames,
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

    # Mark the project as calibrated in metadata.json. The UI gates the
    # "Поднять и начать запись" button on project.calibrationStatus ==
    # "calibrated", so without this the button stays disabled forever after
    # a calib.gpc is produced.
    try:
        project = read_project_metadata(PROJECTS_DIR, project_id)
        if project is not None:
            project.calibration_status = "calibrated"
            write_project_metadata(PROJECTS_DIR, project)
    except Exception as exc:
        print(f"[finalize] warning: could not update calibration_status: {exc}", flush=True)

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

