#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-status}"
TARGET="${2:-all}"
BASE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PID_DIR="$BASE_DIR/.pid"
mkdir -p "$PID_DIR"

BACKEND_PID_FILE="$PID_DIR/backend.pid"
FRONTEND_PID_FILE="$PID_DIR/frontend.pid"

test_port() {
    (echo > /dev/tcp/127.0.0.1/"$1") >/dev/null 2>&1
}

get_status() {
    local name="$1"
    local pid_file="$2"
    local port="$3"
    local running=false
    local pid=""

    if [[ -f "$pid_file" ]]; then
        pid="$(cat "$pid_file")"
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            running=true
        fi
    fi
    if [[ "$running" == "false" ]] && test_port "$port"; then
        running=true
    fi

    printf '%s\t%s\t%s\t%s\n' "$name" "$running" "$pid" "$port"
}

start_backend() {
    local status
    status="$(get_status backend "$BACKEND_PID_FILE" 9000)"
    local running
    running="$(echo "$status" | cut -f2)"
    if [[ "$running" == "true" ]]; then
        echo "Backend already running"
        return
    fi
    echo "Starting backend..."
    nohup python3 "$BASE_DIR/TerraSLAM_relay/system_manager.py" >"$PID_DIR/backend.log" 2>&1 &
    echo $! > "$BACKEND_PID_FILE"
    sleep 2
    if test_port 9000; then
        echo "Backend started (PID $(cat "$BACKEND_PID_FILE"))"
    else
        echo "Backend failed to start"
    fi
}

stop_backend() {
    local status
    status="$(get_status backend "$BACKEND_PID_FILE" 9000)"
    local running pid
    running="$(echo "$status" | cut -f2)"
    pid="$(echo "$status" | cut -f3)"
    if [[ "$running" == "false" ]]; then
        echo "Backend is not running"
        rm -f "$BACKEND_PID_FILE"
        return
    fi
    echo "Stopping backend..."
    if [[ -n "$pid" ]]; then
        kill "$pid" 2>/dev/null || true
    fi
    sleep 1
    rm -f "$BACKEND_PID_FILE"
    echo "Backend stopped"
}

start_frontend() {
    local status
    status="$(get_status frontend "$FRONTEND_PID_FILE" 8080)"
    local running
    running="$(echo "$status" | cut -f2)"
    if [[ "$running" == "true" ]]; then
        echo "Frontend already running"
        return
    fi
    echo "Starting frontend..."
    nohup pnpm --dir "$BASE_DIR/TWA" run dev >"$PID_DIR/frontend.log" 2>&1 &
    echo $! > "$FRONTEND_PID_FILE"
    sleep 3
    if test_port 8080; then
        echo "Frontend started (PID $(cat "$FRONTEND_PID_FILE"))"
    else
        echo "Frontend failed to start"
    fi
}

stop_frontend() {
    local status
    status="$(get_status frontend "$FRONTEND_PID_FILE" 8080)"
    local running pid
    running="$(echo "$status" | cut -f2)"
    pid="$(echo "$status" | cut -f3)"
    if [[ "$running" == "false" ]]; then
        echo "Frontend is not running"
        rm -f "$FRONTEND_PID_FILE"
        return
    fi
    echo "Stopping frontend..."
    if [[ -n "$pid" ]]; then
        kill "$pid" 2>/dev/null || true
    fi
    sleep 1
    rm -f "$FRONTEND_PID_FILE"
    echo "Frontend stopped"
}

case "$ACTION" in
    start)
        if [[ "$TARGET" == "backend" || "$TARGET" == "all" ]]; then start_backend; fi
        if [[ "$TARGET" == "frontend" || "$TARGET" == "all" ]]; then start_frontend; fi
        ;;
    stop)
        if [[ "$TARGET" == "backend" || "$TARGET" == "all" ]]; then stop_backend; fi
        if [[ "$TARGET" == "frontend" || "$TARGET" == "all" ]]; then stop_frontend; fi
        ;;
    restart)
        if [[ "$TARGET" == "backend" || "$TARGET" == "all" ]]; then stop_backend; start_backend; fi
        if [[ "$TARGET" == "frontend" || "$TARGET" == "all" ]]; then stop_frontend; start_frontend; fi
        ;;
    status)
        echo "Backend:  $(get_status backend "$BACKEND_PID_FILE" 9000 | awk -F'\t' '{if($2=="true") print "RUNNING (PID "$3", port "$4")"; else print "STOPPED"}')"
        echo "Frontend: $(get_status frontend "$FRONTEND_PID_FILE" 8080 | awk -F'\t' '{if($2=="true") print "RUNNING (PID "$3", port "$4")"; else print "STOPPED"}')"
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status} [backend|frontend|all]"
        exit 1
        ;;
esac
