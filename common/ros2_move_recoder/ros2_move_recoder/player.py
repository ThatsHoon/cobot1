"""
ros2_move_recoder.player — 매크로 재생 노드 (비동기 모션)
==================================================
smoothed_joints.json 을 읽어 amovesj 비동기 스플라인 모션으로 재생.
amovesj 호출 후 check_motion 폴링으로 완료 대기.

실행:
  ros2 run ros2_move_recoder player <name>
"""

import json
import os
import sys
import time

import rclpy
import DR_init

ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

MACROS_DIR = os.path.expanduser("~/cobot_ws/src/ros2_move_recoder/records")


def _wait_motion(check_motion, poll_sec: float = 0.05):
    """amove* 시작 직후 호출. 모션이 끝날 때까지 폴링."""
    while check_motion():
        time.sleep(poll_sec)


def main(args=None):
    if len(sys.argv) < 2:
        print("사용법: ros2 run ros2_move_recoder player <macro_name>")
        sys.exit(1)
    name = sys.argv[1]

    smooth_path = os.path.join(MACROS_DIR, name, "smooth.json")
    if not os.path.isfile(smooth_path):
        print(f"❌ 평활화 파일 없음: {smooth_path}")
        print(f"   먼저 'ros2 run ros2_move_recoder smoother {name}' 실행하세요.")
        sys.exit(1)

    with open(smooth_path) as f:
        data = json.load(f)
    waypoints = data["waypoints_deg"]
    vel = data.get("vel", 30.0)
    acc = data.get("acc", 60.0)

    print(f"[player] 매크로 로드: {len(waypoints)} waypoint, "
          f"vel={vel}, acc={acc}")

    # 사용자 안전 확인
    try:
        ans = input("[player] 재생을 시작합니다. 작업 공간이 안전한가요? (y/N) ")
    except EOFError:
        ans = ""
    if ans.strip().lower() != "y":
        print("[player] 취소됨")
        return

    # ROS2 노드 + DSR 초기화
    rclpy.init(args=args)
    node = rclpy.create_node("macro_player", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import amovesj, posj, check_motion
        from DSR_ROBOT2 import get_robot_mode, ROBOT_MODE_AUTONOMOUS
    except ImportError as e:
        print(f"[player] DSR_ROBOT2 임포트 실패: {e}")
        rclpy.shutdown()
        return

    # 자동 모드 확인
    try:
        mode = get_robot_mode()
        if mode != ROBOT_MODE_AUTONOMOUS:
            print(f"[player] ⚠️ 로봇이 자동 모드가 아닙니다 (현재 mode={mode})")
            print("[player]    펜던트에서 자동 모드로 전환 후 다시 실행하세요.")
            return
    except Exception as e:
        print(f"[player] 모드 확인 실패(무시하고 진행): {e}")

    # amovesj 호출 — 균일 속도 비동기 재생
    pts = [posj(*w) for w in waypoints]
    print(f"[player] ▶ amovesj(vel={vel}, acc={acc}) — async, 균일 속도")
    t0 = time.monotonic()
    res = amovesj(pts, vel=vel, acc=acc)
    if res != 0:
        print(f"[player] ⚠️ amovesj 시작 반환 코드: {res}")
    else:
        print("[player] ⏳ 모션 진행 중 — check_motion 폴링")
        _wait_motion(check_motion)
        dt = time.monotonic() - t0
        print(f"[player] ✅ 재생 완료 ({dt:.2f}s)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
