#!/bin/bash
cd /opt/main/Trajectory
pkill -f system_manager.py 2>/dev/null
sleep 2
cd TerraSLAM_relay
setsid nohup python3 system_manager.py > /tmp/manager.log 2>&1 < /dev/null &
disown
sleep 7
curl -s -o /dev/null -w "manager http=%{http_code}\n" http://127.0.0.1:9000/api/status
echo "pid: $(pgrep -f system_manager.py)"
