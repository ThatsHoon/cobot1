"""
ros2_move_recoder — 즉시 실행 모션 테스트 (비동기 모션)
=================================================
모든 모션 호출은 amovej/amovel (비동기) 후 check_motion 폴링으로 완료 대기.
실행: ros2 run ros2_move_recoder run
"""

import time
import rclpy
import DR_init

ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id    = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

VEL = 30   # deg/s
ACC = 60   # deg/s²
HOME = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


def _wait_motion(check_motion, poll_sec: float = 0.01):
    """amove* 시작 직후 호출. 모션이 끝날 때까지 폴링."""
    while check_motion():
        time.sleep(poll_sec)


def my_motion(amovej, amovel, posj, posx,
              DR_MV_MOD_REL, DR_TOOL, check_motion):
    """
    ★ 여기에 원하는 동작을 구현하세요 ★
    각 amove* 직후 _wait_motion(check_motion) 호출 필수.
    """
    print("[ros2_move_recoder] 🤖 모션 시작 (async)")

    # 예시 1 — 관절 공간 이동 (J1 +30도)
    amovej(posj(30.0, 0.0, 90.0, 0.0, 90.0, 0.0), vel=VEL, acc=ACC)
    _wait_motion(check_motion)

    # 예시 2 — 툴 기준 +Z 방향 50mm 이동
    amovel(posx(0, 0, 50, 0, 0, 0), vel=100, acc=200,
           mod=DR_MV_MOD_REL, ref=DR_TOOL)
    _wait_motion(check_motion)

    # 예시 3 — 툴 기준 -Z 복귀
    amovel(posx(0, 0, -50, 0, 0, 0), vel=100, acc=200,
           mod=DR_MV_MOD_REL, ref=DR_TOOL)
    _wait_motion(check_motion)

    print("[ros2_move_recoder] 🎬 모션 완료")


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("ros2_move_recoder_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import amovej, amovel, posj, posx, check_motion
        from DSR_ROBOT2 import DR_MV_MOD_REL, DR_TOOL
        from DSR_ROBOT2 import set_velj, set_accj
    except ImportError as e:
        print(f"[ros2_move_recoder] DSR_ROBOT2 임포트 실패: {e}")
        return

    set_velj(VEL)
    set_accj(ACC)

    try:
        my_motion(amovej, amovel, posj, posx,
                  DR_MV_MOD_REL, DR_TOOL, check_motion)
    except Exception as e:
        print(f"[ros2_move_recoder] ⚠️ 모션 오류: {e}")
    finally:
        print("[ros2_move_recoder] 🏠 홈으로 복귀 (async)...")
        amovej(posj(*HOME), vel=VEL, acc=ACC)
        _wait_motion(check_motion)
        print("[ros2_move_recoder] ✅ 완료")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
