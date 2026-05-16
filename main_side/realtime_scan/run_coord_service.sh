#!/bin/bash
# ROBO CHEF — Coordinate Service 실행 스크립트 (메인노드 PC)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/humble/setup.bash

WS_SETUP="${HOME}/cobot_ws/install/setup.bash"
[ -f "$WS_SETUP" ] && source "$WS_SETUP" && echo "[OK] cobot_ws 소싱"

export ROS_DOMAIN_ID=24

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ROBO CHEF — Realtime Coord Scan"
echo "  서비스: /robo_chef/get_coords"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

cd "$SCRIPT_DIR"
python3 realtime_scan/realtime_scan/coord_service.py
