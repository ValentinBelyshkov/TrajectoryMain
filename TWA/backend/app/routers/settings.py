from fastapi import APIRouter, Request, HTTPException
from pydantic import BaseModel
from typing import Dict, Optional
import json
from pathlib import Path

router = APIRouter()

class ORBExtractorSettings(BaseModel):
    nFeatures: int = 1200
    scaleFactor: float = 1.2
    nLevels: int = 8
    iniThFAST: int = 20
    minThFAST: int = 7

class ProtocolConfig(BaseModel):
    id: str
    name: str
    baudRate: Optional[int] = None
    port: Optional[str] = None

class AppSettings(BaseModel):
    protocol: str = "Mavlink"
    transmitter: str = "UART"
    session: str = "Mono"
    orbExtractor: ORBExtractorSettings = ORBExtractorSettings()
    protocolConfigs: Dict[str, ProtocolConfig] = {
        "Mavlink": ProtocolConfig(id="mavlink", name="Mavlink", baudRate=57600, port="/dev/ttyTHS0"),
        "MSP": ProtocolConfig(id="msp", name="MSP", baudRate=115200, port="/dev/ttyTHS0"),
        "Ublox": ProtocolConfig(id="ublox", name="Ublox", baudRate=9600, port="/dev/ttyTHS0"),
        "Custom": ProtocolConfig(id="custom", name="Custom", port="/dev/ttyTHS0")
    }

def get_settings_path(request: Request) -> Path:
    return request.app.state.projects_path / "settings.json"

@router.get("")
async def get_settings(request: Request):
    settings_path = get_settings_path(request)
    if settings_path.exists():
        try:
            with open(settings_path, "r") as f:
                return json.load(f)
        except Exception:
            pass
    
    # Default settings
    return AppSettings().model_dump()

@router.post("")
async def save_settings(request: Request, settings: AppSettings):
    settings_path = get_settings_path(request)
    try:
        # Save as JSON
        with open(settings_path, "w") as f:
            json.dump(settings.model_dump(), f, indent=2)
        
        # Also save as yaml for potential ROS nodes in another container
        yaml_path = request.app.state.projects_path / "settings.yaml"
        with open(yaml_path, "w") as f:
            f.write("# Generated settings for visual odometry\n")
            f.write(f"protocol: {settings.protocol}\n")
            f.write(f"transmitter: {settings.transmitter}\n")
            f.write(f"session_type: {settings.session}\n\n")
            f.write("ORBextractor:\n")
            f.write(f"  nFeatures: {settings.orbExtractor.nFeatures}\n")
            f.write(f"  scaleFactor: {settings.orbExtractor.scaleFactor}\n")
            f.write(f"  nLevels: {settings.orbExtractor.nLevels}\n")
            f.write(f"  iniThFAST: {settings.orbExtractor.iniThFAST}\n")
            f.write(f"  minThFAST: {settings.orbExtractor.minThFAST}\n\n")
            
            f.write("protocols:\n")
            for p_name, p_config in settings.protocolConfigs.items():
                f.write(f"  {p_name}:\n")
                if p_config.baudRate:
                    f.write(f"    baudRate: {p_config.baudRate}\n")
                if p_config.port:
                    f.write(f"    port: \"{p_config.port}\"\n")
                    
        return {"success": True, "settings": settings.model_dump()}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
