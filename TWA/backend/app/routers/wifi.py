# -*- coding: utf-8 -*-
from fastapi import APIRouter, HTTPException, Body
from pydantic import BaseModel
import subprocess, json, re

router = APIRouter(prefix="/wifi", tags=["wifi"])

class ConnectRequest(BaseModel):
    ssid: str
    password: str | None = None   # None > открыта€ сеть

def _run(cmd: list[str]) -> str:
    """¬ыполнить команду и вернуть stdout; при ошибке бросить HTTPException."""
    res = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if res.returncode != 0:
        raise HTTPException(
            status_code=500,
            detail=f" оманда {' '.join(cmd)} завершилась с кодом {res.returncode}: {res.stderr.strip()}"
        )
    return res.stdout.strip()

@router.get("/scan")
async def scan_wifi():
    """
    ¬озвращает список обнаруженных SSID:
    [{ "ssid": "...", "signal": 0-100, "security": "WPA2" }, Е]
    """
    out = _run(["nmcli", "-f", "SSID,SIGNAL,SECURITY", "device", "wifi", "list", "--rescan", "yes"])
    lines = out.splitlines()[1:]          # убираем заголовок
    nets = []
    for ln in lines:
        if not ln.strip():
            continue
        parts = re.split(r'\s{2,}', ln.strip())
        if len(parts) >= 3:
            nets.append({
                "ssid": parts[0],
                "signal": int(parts[1]) if parts[1].isdigit() else 0,
                "security": parts[2]
            })
    return {"networks": nets}

@router.post("/connect")
async def connect_wifi(req: ConnectRequest = Body(...)):
    """
    ѕодключаетс€ к выбранной сети.
    ≈сли соединение уже существует Ц просто активирует его.
    """
    cmd = ["nmcli", "device", "wifi", "connect", req.ssid]
    if req.password:
        cmd.extend(["password", req.password])
    cmd.extend(["autoconnect", "yes"])   # сохран€ем дл€ авто-подключени€ при загрузке
    try:
        _run(cmd)
    except HTTPException as exc:
        # ≈сли соединение уже есть, пытаемс€ просто подн€ть его
        if "already active" not in str(exc.detail):
            raise
        _run(["nmcli", "connection", "up", req.ssid])
        return {"status": "re-activated", "ssid": req.ssid}
    return {"status": "connected", "ssid": req.ssid}

@router.get("/status")
async def wifi_status():
    """
    “екущее активное Wi-Fi соединение (может быть несколько, но обычно одно).
    """
    out = _run(["nmcli", "-g", "NAME,STATE,IP4.ADDRESS", "connection", "show", "--active"])
    active = []
    for ln in out.splitlines():
        if not ln:
            continue
        name, state, ip = ln.split(":")
        active.append({"name": name, "state": state, "ip": ip})
    return {"active_connections": active}
