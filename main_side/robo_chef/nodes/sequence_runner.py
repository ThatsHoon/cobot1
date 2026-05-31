"""sequence_runner — /recipe(주문 잡) 수신 → 세그먼트 체인 재생.

DSR 소유 노드. 재생은 ros2_move_recoder.playback.play_segment 위임.
실패 시 즉시 정지 + ERROR, unlock_system(Trigger) 으로만 복귀.
"""
import os
import json
import threading

import rclpy
import DR_init
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ros2_move_recoder.playback import play_segment
import cooking_core as cc

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"
RECORDS_DIR = os.path.expanduser("~/cobot_ws/src/ros2_move_recoder/records")

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class SequenceRunner(Node):
    def __init__(self):
        super().__init__("sequence_runner")
        self.state = "IDLE"
        self._abort = threading.Event()
        self._lock = threading.Lock()
        self._gripper = self._init_gripper()
        self.init_dsr()
        self.status_pub = self.create_publisher(String, "/cooking_status", 10)
        self.create_subscription(String, "/recipe", self._on_recipe, 10)
        self.create_service(Trigger, "unlock_system", self._on_unlock)
        self.get_logger().info("🍳 sequence_runner ready (state=IDLE)")

    def _init_gripper(self):
        try:
            from ros2_move_recoder.onrobot import RG
            ip = os.environ.get("GRIPPER_IP", "192.168.1.1")
            port = int(os.environ.get("GRIPPER_PORT", "502"))
            gtype = os.environ.get("GRIPPER_TYPE", "rg2")
            return RG(gtype, ip, port)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"그리퍼 init 실패(그리퍼 없이 진행): {e}")
            return None

    def init_dsr(self):
        from DSR_ROBOT2 import (set_tool, set_tcp, ROBOT_MODE_MANUAL,
                                ROBOT_MODE_AUTONOMOUS, set_robot_mode)
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    def _emit(self, status: dict):
        m = String()
        m.data = json.dumps(status)
        self.status_pub.publish(m)

    def _seg_path(self, seg: str) -> str:
        return os.path.join(RECORDS_DIR, seg, "smooth.json")

    def _play(self, smooth_path: str) -> bool:
        res = play_segment(smooth_path, gripper=self._gripper,
                           require_autonomous=True, abort_event=self._abort,
                           logger=self.get_logger())
        return res.ok

    def _missing_segments(self, jobs: list) -> list:
        """모든 job 의 모든 segment 의 smooth.json 존재 검사. 누락된 seg 이름 반환.
        실제 모션 시작 전에 fail-fast 하기 위한 pre-flight 게이트."""
        missing = []
        for job in jobs:
            for seg in (job.get("segments") or []):
                if not os.path.isfile(self._seg_path(seg)):
                    missing.append(seg)
        return missing

    def _on_recipe(self, msg: String):
        # NOTE: /recipe subscription MUST remain in a MutuallyExclusiveCallbackGroup
        # (rclpy default). _lock makes the IDLE→EXECUTING transition atomic so
        # correctness does not depend on that scheduling property.
        with self._lock:
            if self.state != "IDLE":
                self.get_logger().warn("state!=IDLE — /recipe 무시")
                return
            try:
                job = json.loads(msg.data)
                order_id = job["order_id"]
                jobs = job["jobs"]
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"잘못된 /recipe: {e}")
                return
            # Pre-flight — segment 파일이 하나라도 없으면 모션 시작 안 함.
            # 실행 중 폭발하면 emergency stop + ERROR 잠금이 되므로 사전 차단 가치 큼.
            missing = self._missing_segments(jobs)
            if missing:
                err_msg = f"missing segments: {', '.join(missing)}"
                self.get_logger().error(f"❌ pre-flight 실패 ({order_id}): {err_msg}")
                self._emit({"state": "ERROR", "order_id": order_id,
                            "recipe_id": "", "item_index": 0, "item_total": 0,
                            "qty_index": 0, "qty_total": 0, "segment_name": "",
                            "segment_index": 0, "segment_total": 0,
                            "error_msg": err_msg})
                return
            self.state = "EXECUTING"
            self._abort.clear()
        try:
            final = cc.run_jobs(order_id, jobs, play_fn=self._play,
                                emit_fn=self._emit, seg_path=self._seg_path)
            new_state = "ERROR" if final["state"] == "ERROR" else "IDLE"
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"run_jobs 예외 → ERROR: {e}")
            self._emit({"state": "ERROR", "order_id": order_id,
                        "recipe_id": "", "item_index": 0, "item_total": 0,
                        "qty_index": 0, "qty_total": 0, "segment_name": "",
                        "segment_index": 0, "segment_total": 0,
                        "error_msg": f"run_jobs exception: {e}"})
            new_state = "ERROR"
        with self._lock:
            self.state = new_state

    def _on_unlock(self, request, response):
        if self.state == "ERROR":
            self.state = "IDLE"
            self._emit({"state": "IDLE", "order_id": "", "recipe_id": "",
                        "item_index": 0, "item_total": 0, "qty_index": 0,
                        "qty_total": 0, "segment_name": "",
                        "segment_index": 0, "segment_total": 0,
                        "error_msg": ""})
            response.success = True
            response.message = "unlocked → IDLE"
        else:
            response.success = False
            response.message = f"state={self.state} (ERROR 아님)"
        return response


def main(args=None):
    rclpy.init(args=args)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    dsr_node = Node("dsr_helper_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    try:
        import DSR_ROBOT2  # noqa: F401
    except Exception as e:  # noqa: BLE001
        print(f"DSR_ROBOT2 Load Error: {e}")
    executor = MultiThreadedExecutor(num_threads=4)
    node = SequenceRunner()
    executor.add_node(dsr_node)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._abort.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
