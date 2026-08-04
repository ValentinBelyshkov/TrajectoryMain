param(
    [ValidateSet("start","stop","restart","status")]
    [string]$Action = "status",
    [ValidateSet("backend","frontend","all")]
    [string]$Target = "all"
)

$ErrorActionPreference = "Stop"
$BaseDir = Split-Path -Parent $MyInvocation.MyCommand.Path | Split-Path -Parent
Set-Location $BaseDir

$PidDir = Join-Path $BaseDir ".pid"
$BackendPidFile = Join-Path $PidDir "backend.pid"
$FrontendPidFile = Join-Path $PidDir "frontend.pid"

function Test-Port($port) {
    $conn = New-Object System.Net.Sockets.TcpClient
    try {
        $conn.Connect("127.0.0.1", $port)
        $conn.Close()
        return $true
    } catch {
        return $false
    }
}

function Get-ServiceStatus($name, $pidFile, $port) {
    $running = $false
    $pid = $null
    if (Test-Path $pidFile) {
        $pid = Get-Content $pidFile -ErrorAction SilentlyContinue
        if ($pid -and (Get-Process -Id $pid -ErrorAction SilentlyContinue)) {
            $running = $true
        }
    }
    if (-not $running -and (Test-Port $port)) {
        $running = $true
    }
    [PSCustomObject]@{
        Name = $name
        Running = $running
        Pid = $pid
        Port = $port
    }
}

function Start-Backend {
    if ((Get-ServiceStatus "backend" $BackendPidFile 9000).Running) {
        Write-Host "Backend already running"
        return
    }
    Write-Host "Starting backend..."
    $p = Start-Process -FilePath "python" -ArgumentList "TerraSLAM_relay\system_manager.py" -WorkingDirectory $BaseDir -WindowStyle Hidden -PassThru
    Set-Content -Path $BackendPidFile -Value $p.Id
    Start-Sleep -Seconds 2
    if ((Test-Port 9000)) {
        Write-Host "Backend started (PID $($p.Id))"
    } else {
        Write-Host "Backend failed to start"
    }
}

function Stop-Backend {
    $status = Get-ServiceStatus "backend" $BackendPidFile 9000
    if (-not $status.Running) {
        Write-Host "Backend is not running"
        if (Test-Path $BackendPidFile) { Remove-Item $BackendPidFile }
        return
    }
    Write-Host "Stopping backend..."
    if ($status.Pid) {
        Stop-Process -Id $status.Pid -Force -ErrorAction SilentlyContinue
    } else {
        $proc = Get-NetTCPConnection -LocalPort 9000 -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty OwningProcess
        if ($proc) { Stop-Process -Id $proc -Force -ErrorAction SilentlyContinue }
    }
    Start-Sleep -Seconds 1
    if (Test-Path $BackendPidFile) { Remove-Item $BackendPidFile }
    Write-Host "Backend stopped"
}

function Start-Frontend {
    if ((Get-ServiceStatus "frontend" $FrontendPidFile 8080).Running) {
        Write-Host "Frontend already running"
        return
    }
    Write-Host "Starting frontend..."
    $p = Start-Process -FilePath "pnpm" -ArgumentList "--dir","TWA","run","dev" -WorkingDirectory $BaseDir -WindowStyle Hidden -PassThru
    Set-Content -Path $FrontendPidFile -Value $p.Id
    Start-Sleep -Seconds 3
    if ((Test-Port 8080)) {
        Write-Host "Frontend started (PID $($p.Id))"
    } else {
        Write-Host "Frontend failed to start"
    }
}

function Stop-Frontend {
    $status = Get-ServiceStatus "frontend" $FrontendPidFile 8080
    if (-not $status.Running) {
        Write-Host "Frontend is not running"
        if (Test-Path $FrontendPidFile) { Remove-Item $FrontendPidFile }
        return
    }
    Write-Host "Stopping frontend..."
    if ($status.Pid) {
        Stop-Process -Id $status.Pid -Force -ErrorAction SilentlyContinue
    } else {
        $proc = Get-NetTCPConnection -LocalPort 8080 -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty OwningProcess
        if ($proc) { Stop-Process -Id $proc -Force -ErrorAction SilentlyContinue }
    }
    Start-Sleep -Seconds 1
    if (Test-Path $FrontendPidFile) { Remove-Item $FrontendPidFile }
    Write-Host "Frontend stopped"
}

switch ($Action) {
    "start" {
        if ($Target -eq "backend" -or $Target -eq "all") { Start-Backend }
        if ($Target -eq "frontend" -or $Target -eq "all") { Start-Frontend }
    }
    "stop" {
        if ($Target -eq "backend" -or $Target -eq "all") { Stop-Backend }
        if ($Target -eq "frontend" -or $Target -eq "all") { Stop-Frontend }
    }
    "restart" {
        if ($Target -eq "backend" -or $Target -eq "all") { Stop-Backend; Start-Backend }
        if ($Target -eq "frontend" -or $Target -eq "all") { Stop-Frontend; Start-Frontend }
    }
    "status" {
        $backend = Get-ServiceStatus "backend" $BackendPidFile 9000
        $frontend = Get-ServiceStatus "frontend" $FrontendPidFile 8080
        Write-Host ("Backend  : {0}" -f $(if($backend.Running){"RUNNING (PID $($backend.Pid), port 9000)"}else{"STOPPED"}))
        Write-Host ("Frontend : {0}" -f $(if($frontend.Running){"RUNNING (PID $($frontend.Pid), port 8080)"}else{"STOPPED"}))
    }
}
