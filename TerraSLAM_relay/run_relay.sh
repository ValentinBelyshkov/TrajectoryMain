#!/bin/bash
# Читаем путь, который backend положил в контейнер перед стартом
CALIB_PATH=$(cat /tmp/terraslam_relay_calib_path 2>/dev/null || echo "")

if [ -z "$CALIB_PATH" ]; then
    echo "ERROR: /tmp/terraslam_relay_calib_path not set"
    exit 1
fi

source /opt/ros/humble/setup.bash
source /home/orb/colcon_ws/install/setup.bash
cd /home/orb/TerraSLAM_relay
exec python3 gps_client.py "$CALIB_PATH"
