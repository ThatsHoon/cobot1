#!/bin/bash
# ──────────────────────────────────────────────────────────────────
#  ROBO CHEF — Main Node Order Receiver 실행 스크립트
#  메인노드(로봇 팔 연결 PC)에서 실행한다.
#
#  사전 조건:
#    1. ros2 bringup 실행 중  (ros2 launch dsr_bringup2 ...)
#    2. pip install firebase-admin  (선택)
#    3. serviceAccountKey.json  (이 스크립트와 같은 디렉터리)
# ──────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ── ROS2 환경 소싱 ─────────────────────────────────────────────────
source /opt/ros/humble/setup.bash

# doosan-robot2 워크스페이스 소싱 (경로가 다를 경우 수정)
WS_SETUP="${HOME}/cobot_ws/install/setup.bash"
if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
    echo "[OK] cobot_ws 소싱 완료"
else
    echo "[WARN] cobot_ws install/setup.bash 없음 — 기본 ROS2만 사용"
fi

# ── 환경 변수 ────────────────────────────────────────────────────────
export ROS_DOMAIN_ID=24
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH}"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ROBO CHEF — Order Receiver (Main Node)"
echo "  구독 토픽: /robo_chef/order_request"
echo "  역할: 주문 수신 시 레시피 JSON 출력 (log only)"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

cd "$SCRIPT_DIR"
python3 realtime_scan/realtime_scan/order_receiver.py
