"""
ros2_move_recoder.recorder — 관절 좌표 기록 노드
==========================================
티칭펜던트로 로봇을 직접 조작하는 동안 /dsr01/joint_states 를 고주기로 기록.
Enter 키로 시작/정지, JSON 파일로 저장.

실행:
  ros2 run ros2_move_recoder recorder <name>
"""

import json
import math
import os
import sys
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

ROBOT_ID = "dsr01"

# 저장 경로
MACROS_DIR = os.path.expanduser("~/cobot_ws/src/ros2_move_recoder/records")


class RecorderNode(Node):
    def __init__(self, name: str):
        super().__init__("macro_recorder", namespace=ROBOT_ID)
        self.macro_name = name
        self.cb_group = ReentrantCallbackGroup()

        self._buffer_t: list[float] = []     # monotonic ns
        self._buffer_q: list[list[float]] = []   # joint deg, [J1..J6]
        self._lock = threading.Lock()
        self._recording = False
        self._t0 = 0.0

        # ★ BEST_EFFORT 필수 — RELIABLE 로 두면 DDS backpressure 로
        #   메시지가 0.3Hz 만 들어오는 현상이 발생함 (실측 검증).
        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            JointState, "/dsr01/joint_states",
            self._on_joint, qos, callback_group=self.cb_group,
        )
        self.get_logger().info("recorder 노드 준비 완료")

    def _on_joint(self, msg: JointState):
        if not self._recording:
            return
        # 이름 → 위치 매핑 후 J1~J6 순서로 추출
        try:
            name_to_pos = dict(zip(msg.name, msg.position))
            joint_deg = [round(math.degrees(float(name_to_pos[f"joint_{i}"])), 4)
                         for i in range(1, 7)]
        except (KeyError, ValueError):
            return

        t_now = time.monotonic()
        with self._lock:
            self._buffer_t.append(t_now)
            self._buffer_q.append(joint_deg)

    def start(self):
        with self._lock:
            self._buffer_t.clear()
            self._buffer_q.clear()
            self._t0 = time.monotonic()
            self._recording = True

    def stop(self):
        with self._lock:
            self._recording = False
        return self._dump()

    def _dump(self) -> str:
        with self._lock:
            ts = list(self._buffer_t)
            qs = list(self._buffer_q)

        if not qs:
            self.get_logger().warn("기록된 데이터가 없습니다")
            return ""

        t0 = ts[0]
        timestamps_ms = [int((t - t0) * 1000) for t in ts]
        duration_sec = (ts[-1] - t0)
        rate_hz_avg = len(qs) / duration_sec if duration_sec > 0 else 0.0

        action_dir = os.path.join(MACROS_DIR, self.macro_name)
        os.makedirs(action_dir, exist_ok=True)
        out_path = os.path.join(action_dir, "raw.json")

        payload = {
            "timestamps_ms": timestamps_ms,
            "joints_deg": qs,
            "rate_hz_avg": round(rate_hz_avg, 2),
            "duration_sec": round(duration_sec, 3),
            "samples": len(qs),
            "recorded_at": datetime.now(timezone.utc).isoformat(),
        }
        with open(out_path, "w") as f:
            json.dump(payload, f, indent=2)

        self.get_logger().info(
            f"저장 완료: {out_path}  (샘플 {len(qs)}개, "
            f"{duration_sec:.2f}s, 평균 {rate_hz_avg:.1f}Hz)"
        )
        return out_path


def _wait_enter(prompt: str):
    print(prompt, end="", flush=True)
    sys.stdin.readline()


def _input_thread(node: RecorderNode, done_event: threading.Event):
    try:
        _wait_enter("[recorder] Enter 를 눌러 기록 시작 → ")
        node.start()
        print("[recorder] ▶ 기록 중... Enter 를 다시 눌러 종료")
        _wait_enter("")
        node.stop()
    finally:
        done_event.set()


def main(args=None):
    if len(sys.argv) < 2:
        print("사용법: ros2 run ros2_move_recoder recorder <macro_name>")
        sys.exit(1)
    name = sys.argv[1]

    rclpy.init(args=args)
    node = RecorderNode(name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    done = threading.Event()
    threading.Thread(target=_input_thread, args=(node, done), daemon=True).start()

    try:
        while rclpy.ok() and not done.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
