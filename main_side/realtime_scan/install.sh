#!/bin/bash
# ──────────────────────────────────────────────────────────────
#  ROBO CHEF — main_node → cobot_ws 설치 스크립트
#  메인노드 PC에서 실행.
#
#  수행 작업:
#    1. robo_chef 패키지를 ~/cobot_ws/src/ 에 심볼릭 링크
#    2. colcon build --packages-select robo_chef
#    3. 사용법 안내
# ──────────────────────────────────────────────────────────────

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$HOME/cobot_ws"
SRC_DIR="$WS_DIR/src"
PKG_NAME="robo_chef"

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ROBO CHEF — 패키지 설치"
echo "  소스: $SCRIPT_DIR"
echo "  대상: $SRC_DIR/$PKG_NAME"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 1. 워크스페이스 확인
if [ ! -d "$WS_DIR" ]; then
    echo "[오류] $WS_DIR 없음"
    exit 1
fi
mkdir -p "$SRC_DIR"

# 2. 심볼릭 링크 (이미 있으면 삭제 후 재생성)
TARGET="$SRC_DIR/$PKG_NAME"
if [ -L "$TARGET" ]; then
    rm "$TARGET"
    echo "[OK] 기존 링크 제거"
elif [ -d "$TARGET" ]; then
    echo "[경고] $TARGET 가 이미 실제 디렉터리로 존재합니다."
    echo "       수동으로 제거 후 다시 실행하세요."
    exit 1
fi

ln -s "$SCRIPT_DIR" "$TARGET"
echo "[OK] 심볼릭 링크 생성: $TARGET → $SCRIPT_DIR"

# 3. ROS2 환경 소싱
source /opt/ros/humble/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

# 4. 빌드
echo ""
echo "[빌드 시작] colcon build --packages-select $PKG_NAME"
cd "$WS_DIR"
colcon build --packages-select "$PKG_NAME" --symlink-install

# 5. 완료
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  설치 완료!"
echo ""
echo "  다음 명령으로 실행하세요:"
echo ""
echo "  source ~/cobot_ws/install/setup.bash"
echo ""
echo "  ros2 run robo_chef coord_service    # 좌표 스캔 서비스"
echo "  ros2 run robo_chef order_receiver   # 주문 수신 + 레시피 실행"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
