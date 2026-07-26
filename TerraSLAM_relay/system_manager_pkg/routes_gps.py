"""
GPS Sender routes:
  GET  /api/v1/gps/calib      — read calib.gpc content
  POST /api/v1/gps/calib      — save calib.gpc content
  GET  /api/v1/gps/position   — compute lat/lon/alt from last /camera_pose + calib.gpc transform
"""
import os
import time
from typing import Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from . import ros_pose_subscriber

router = APIRouter()

CALIB_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "Serial", "calib.gpc"
)


class CalibContent(BaseModel):
    content: str


def _apply_calib(filepath: str, x: float, y: float, z: float):
    """Parse calib.gpc and apply least-squares affine transform to get lat/lon/alt."""
    try:
        import numpy as np
    except ImportError:
        raise RuntimeError("numpy required")

    pts = []
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("+proj"):
                continue
            tok = line.split()
            if len(tok) >= 5:
                px, py, pz = float(tok[0]), float(tok[1]), float(tok[2])
                lon = float(tok[3])
                lat = float(tok[4])
                alt = float(tok[5]) if len(tok) > 5 else 0.0
                pts.append((px, py, pz, lon, lat, alt))

    if len(pts) < 3:
        raise ValueError(f"Need ≥3 calibration points, got {len(pts)}")

    A = np.array([[p[0], p[1], p[2], 1.0] for p in pts])
    lats = np.array([p[4] for p in pts])
    lons = np.array([p[3] for p in pts])
    alts = np.array([p[5] for p in pts])

    coeff_lat, *_ = np.linalg.lstsq(A, lats, rcond=None)
    coeff_lon, *_ = np.linalg.lstsq(A, lons, rcond=None)

    v = np.array([x, y, z, 1.0])
    lat_out = float(np.dot(coeff_lat, v))
    lon_out = float(np.dot(coeff_lon, v))

    if np.std(alts) < 1e-6:
        # All calibration points at the same altitude — can't determine scale.
        # x is the altitude axis: more negative x = higher drone (nadir camera).
        # Best estimate: mean calibration altitude shifted by the x offset from
        # the mean calibration x, negated because altitude grows as x decreases.
        mean_px = float(np.mean([p[0] for p in pts]))
        alt_out = float(np.mean(alts)) - (x - mean_px)
    else:
        coeff_alt, *_ = np.linalg.lstsq(A, alts, rcond=None)
        alt_out = float(np.dot(coeff_alt, v))

    return lat_out, lon_out, alt_out


@router.get("/api/v1/gps/calib")
def get_calib():
    """Return the raw text content of calib.gpc."""
    try:
        with open(CALIB_PATH, "r", encoding="utf-8") as f:
            content = f.read()
        return {"success": True, "content": content, "path": CALIB_PATH}
    except FileNotFoundError:
        raise HTTPException(404, detail=f"calib.gpc not found at {CALIB_PATH}")
    except Exception as e:
        raise HTTPException(500, detail=str(e))


@router.post("/api/v1/gps/calib")
def save_calib(req: CalibContent):
    """Overwrite calib.gpc with new content."""
    try:
        with open(CALIB_PATH, "w", encoding="utf-8") as f:
            f.write(req.content)
        return {"success": True, "path": CALIB_PATH}
    except Exception as e:
        raise HTTPException(500, detail=str(e))


@router.get("/api/v1/gps/position")
def get_gps_position():
    """
    Read the last /camera_pose position from the persistent subscriber,
    apply the calib.gpc affine transform, and return lat/lon/alt.
    """
    snap = ros_pose_subscriber.state.snapshot()
    pos = snap.get("last_position")
    last_seen: Optional[float] = snap.get("last_seen")

    if pos is None:
        return {
            "success": False,
            "error": "No pose received yet from /camera_pose",
            "lat": None, "lon": None, "alt": None,
            "pose_age_seconds": None,
        }

    x, y, z = pos
    age = (time.time() - last_seen) if last_seen else None

    try:
        lat, lon, alt = _apply_calib(CALIB_PATH, x, y, z)
        return {
            "success": True,
            "lat": lat,
            "lon": lon,
            "alt": round(alt, 3),
            "pose_x": x, "pose_y": y, "pose_z": z,
            "pose_age_seconds": round(age, 2) if age is not None else None,
        }
    except FileNotFoundError:
        return {
            "success": False,
            "error": "calib.gpc not found — save it first",
            "lat": None, "lon": None, "alt": None,
            "pose_age_seconds": round(age, 2) if age is not None else None,
        }
    except Exception as e:
        return {
            "success": False,
            "error": str(e),
            "lat": None, "lon": None, "alt": None,
            "pose_age_seconds": round(age, 2) if age is not None else None,
        }
