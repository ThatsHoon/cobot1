"""ros2_move_recoder.player — smooth.json 재생 CLI (playback 코어 위임).

  ros2 run ros2_move_recoder player <name> [--yes]
"""
import os
import sys
import rclpy
import DR_init
from ros2_move_recoder.playback import play_segment

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
MACROS_DIR = os.path.expanduser("~/cobot_ws/src/ros2_move_recoder/records")


def main(args=None):
    argv = [a for a in sys.argv[1:] if a]
    yes = "--yes" in argv
    names = [a for a in argv if not a.startswith("-")]
    if not names:
        print("사용법: ros2 run ros2_move_recoder player <name> [--yes]")
        sys.exit(1)
    name = names[0]
    smooth_path = os.path.join(MACROS_DIR, name, "smooth.json")
    if not os.path.isfile(smooth_path):
        print(f"❌ 평활화 파일 없음: {smooth_path}")
        sys.exit(1)
    if not yes:
        try:
            if input("[player] 재생 시작? 작업 공간 안전한가요? (y/N) ").strip().lower() != "y":
                print("[player] 취소됨")
                return
        except EOFError:
            print("[player] 비대화 환경 — --yes 필요")
            return

    rclpy.init(args=args)
    node = rclpy.create_node("macro_player", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        res = play_segment(smooth_path, require_autonomous=True)
        print(f"[player] {'✅ 완료' if res.ok else '❌ 실패'}: "
              f"{res.error or ''} ({res.duration_sec:.2f}s)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
