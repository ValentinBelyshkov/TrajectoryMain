#!/bin/bash
#
# terraslam_service.sh — остановка / перезапуск / статус службы TerraSLAM.
#
# Служба TerraSLAM поднимается как Docker-контейнер (имя "TerraSLAM",
# см. docker-compose.yml -> services.TerraSLAM, restart: unless-stopped).
# Скрипт сначала пытается управлять через docker; если контейнер не
# найден — откатывается на systemd (sudo systemctl).
#
# Использование:
#   ./terraslam_service.sh status
#   ./terraslam_service.sh stop
#   ./terraslam_service.sh restart
#   ./terraslam_service.sh start
#
set -u

CONTAINER="TerraSLAM"
SERVICE="terraslam"

action="${1:-status}"

log() { echo "[terraslam] $*"; }
die() { echo "[terraslam][ERROR] $*" >&2; exit 1; }

# --- Выбор бэкенда (docker vs systemd) ---
use_docker=0
if command -v docker >/dev/null 2>&1; then
    if docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "$CONTAINER"; then
        use_docker=1
    fi
fi

docker_backend() {
    case "$action" in
        status)
            log "backend=docker container=$CONTAINER"
            docker ps --filter "name=^${CONTAINER}$" --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'
            ;;
        stop)
            log "stopping container $CONTAINER ..."
            docker stop "$CONTAINER"
            ;;
        start)
            log "starting container $CONTAINER ..."
            docker start "$CONTAINER"
            ;;
        restart)
            log "restarting container $CONTAINER ..."
            docker restart "$CONTAINER"
            ;;
        *)
            die "unknown action: $action (use: status|start|stop|restart)"
            ;;
    esac
}

systemd_backend() {
    case "$action" in
        status)
            log "backend=systemd service=$SERVICE"
            sudo systemctl status "$SERVICE" --no-pager || true
            ;;
        stop)
            log "stopping service $SERVICE ..."
            sudo systemctl stop "$SERVICE"
            ;;
        start)
            log "starting service $SERVICE ..."
            sudo systemctl start "$SERVICE"
            ;;
        restart)
            log "restarting service $SERVICE ..."
            sudo systemctl restart "$SERVICE"
            ;;
        *)
            die "unknown action: $action (use: status|start|stop|restart)"
            ;;
    esac
}

if [ "$use_docker" -eq 1 ]; then
    docker_backend
else
    # Если docker есть, но контейнер не найден — сообщаем и пробуем systemd.
    if command -v docker >/dev/null 2>&1; then
        log "container '$CONTAINER' not found via docker, falling back to systemd"
    fi
    command -v systemctl >/dev/null 2>&1 || die "neither docker container '$CONTAINER' nor systemctl available"
    systemd_backend
fi
