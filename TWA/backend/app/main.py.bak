import os
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
from app.routers import projects, calibration, telemetry, control, video, settings
from pathlib import Path

load_dotenv()

# Get the backend directory path
BACKEND_DIR = Path(__file__).parent.parent

def get_data_path(env_var: str, default_name: str) -> Path:
    """Get data path - use env variable or create in backend directory."""
    env_path = os.getenv(env_var)
    if env_path:
        return Path(env_path)
    data_dir = BACKEND_DIR / "data"
    return data_dir / default_name

@asynccontextmanager
async def lifespan(app: FastAPI):
    video_path = get_data_path("VIDEO_SAVE_PATH", "uploads")
    calibration_path = get_data_path("CALIBRATION_PATH", "calibrations")
    projects_path = get_data_path("PROJECTS_PATH", "projects")
    
    video_path.mkdir(parents=True, exist_ok=True)
    calibration_path.mkdir(parents=True, exist_ok=True)
    projects_path.mkdir(parents=True, exist_ok=True)
    
    app.state.video_path = video_path
    app.state.calibration_path = calibration_path
    app.state.projects_path = projects_path
    yield

app = FastAPI(title="Visual Odometry API", lifespan=lifespan)

# Parse origins from env and clean them (remove whitespace)
origins_str = os.getenv(
    "CORS_ORIGINS", 
    "http://localhost:3000,http://127.0.0.1:3000,http://localhost:8080,http://localhost:5173"
)
# CRITICAL: Strip whitespace from each origin!
origins = [origin.strip() for origin in origins_str.split(",") if origin.strip()]
origins = [
    "http://localhost:3000",
    "http://127.0.0.1:3000",
    "http://localhost:5173",
    "http://localhost:8000",
    "ws://localhost:3000",  # WebSocket origin
    "ws://localhost:8000",
]
print(f"Configured CORS origins: {origins}")  # Debug log

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,  # Use the cleaned origins list
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
    expose_headers=["*"],
    max_age=3600,
)

# Include routers
app.include_router(projects.router, prefix="/api/projects", tags=["projects"])
app.include_router(calibration.router, prefix="/api/projects", tags=["calibration"])
app.include_router(telemetry.router, prefix="/api/telemetry", tags=["telemetry"])
app.include_router(control.router, prefix="/api/control", tags=["control"])
app.include_router(video.router, prefix="/api/video", tags=["video"])
app.include_router(settings.router, prefix="/api/settings", tags=["settings"])

@app.get("/health")
async def health_check():
    return {"status": "healthy"}
