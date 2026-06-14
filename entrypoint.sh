#!/bin/bash
set -e

# Export ROS2 paths globally for ALL child processes
export PATH="/opt/ros/humble/bin:$PATH"
export LD_LIBRARY_PATH="/opt/ros/humble/lib:$LD_LIBRARY_PATH"
export COLCON_PREFIX_PATH="/home/orb/colcon_ws/install:$COLCON_PREFIX_PATH"

# Source for current shell (interactive commands)
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /home/orb/colcon_ws/install/setup.bash ] && source /home/orb/colcon_ws/install/setup.bash

export LD_LIBRARY_PATH="/home/orb/ORB_SLAM3/lib:/home/orb/colcon_ws/install/lib:/opt/ros/humble/lib:$LD_LIBRARY_PATH"
# Export environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export QT_X11_NO_MITSHM=1
export NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
export NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-all}

# === X11 / DISPLAY настройки ===
# Если DISPLAY не передан извне — пробуем найти активный
if [ -z "$DISPLAY" ]; then
    # Проверяем, есть ли сокеты X11 внутри контейнера
    if [ -S "/tmp/.X11-unix/X0" ]; then
        export DISPLAY=":0"
    elif [ -S "/tmp/.X11-unix/X1" ]; then
        export DISPLAY=":1"
    elif [ -S "/tmp/.X11-unix/X10" ]; then
        export DISPLAY=":10"
    else
        # Fallback — пробуем :0, но приложение само упадёт если не работает
        export DISPLAY=":0"
    fi
fi

# XAUTHORITY: если не передана, создаём из переменной окружения или файла
if [ -z "$XAUTHORITY" ] || [ ! -f "$XAUTHORITY" ]; then
    # Если есть XAUTH_DATA (base64 cookie), декодируем
    if [ -n "$XAUTH_DATA" ]; then
        export XAUTHORITY="/tmp/.docker.xauth"
        echo "$XAUTH_DATA" | base64 -d > "$XAUTHORITY"
        chmod 600 "$XAUTHORITY"
    else
        # Иначе разрешаем любой доступ (только для изолированных систем!)
        # Лучше пробросить реальный xauth с хоста
        export XAUTHORITY="/tmp/.docker.xauth"
        touch "$XAUTHORITY"
        xauth + "$DISPLAY" 2>/dev/null || true
    fi
fi

mkdir -p /home/orb/Database
chmod -R 777 /home/orb/Database
chown -R orb:orb /home/orb/Database 2>/dev/null || true
tail -f /dev/null
#exec python3 /home/orb/TerraSLAM_relay/system_manager.py
