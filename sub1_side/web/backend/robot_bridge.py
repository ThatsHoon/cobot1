"""
ROS2 ↔ Flask 브릿지 (rclpy 네이티브)

기존 `ros2` CLI subprocess 호출(매 호출 100~300 ms 오버헤드 + 텍스트 파싱)을
백그라운드 rclpy 노드 + 미리 생성된 service client 패턴으로 대체.

폴링 루프(_monitor_loop)는 폐기 — coord_service 가 이미 Firebase `telemetry/*`,
`robot_state` 에 직접 발행하고 fk_worker 가 TCP 를 재계산해 `/robot_state.tcp_position`
에 merge-update 한다. 두 경로가 같은 데이터를 중복 publish 하던 구조를 단일 경로로 정리.

외부 인터페이스(stop_robot, move_joint, set_robot_mode_autonomous,
start_state_monitor, stop_state_monitor)는 시그니처를 그대로 유지해
Flask 라우터(app.py)는 손대지 않는다. monitor 함수 둘은 호환을 위한 no-op.
"""
import threading
import time

import rclpy
from rclpy.node import Node

from dsr_msgs2.srv import MoveStop, MoveJoint, SetRobotMode

import config
import firebase_client as fb


_node: Node | None = None
_cli_stop = None
_cli_move_joint = None
_cli_set_mode = None
_spin_thread: threading.Thread | None = None
_ready = threading.Event()


def _spin_worker():
    global _node, _cli_stop, _cli_move_joint, _cli_set_mode

    # rclpy.init() 은 app.py startup() 에서 단일 진입점으로 호출되지만,
    # 단독 실행/import 순서 변동에 대비한 방어적 init.
    if not rclpy.ok():
        rclpy.init()

    _node = rclpy.create_node("rc_robot_bridge")
    ns = config.ROS2_NAMESPACE
    _cli_stop       = _node.create_client(MoveStop,     f"/{ns}/motion/move_stop")
    _cli_move_joint = _node.create_client(MoveJoint,    f"/{ns}/motion/move_joint")
    _cli_set_mode   = _node.create_client(SetRobotMode, f"/{ns}/system/set_robot_mode")
    _ready.set()
    try:
        while rclpy.ok():
            rclpy.spin_once(_node, timeout_sec=0.5)
    finally:
        _node.destroy_node()


def start():
    """app.py startup() 에서 호출. 백그라운드 노드 1개를 띄운다."""
    global _spin_thread
    if _spin_thread and _spin_thread.is_alive():
        return
    _spin_thread = threading.Thread(
        target=_spin_worker, daemon=True, name="rc_robot_bridge"
    )
    _spin_thread.start()
    _ready.wait(timeout=5.0)


def _call(cli, req, timeout: float):
    """미리 생성된 클라이언트로 service 호출. 백그라운드 thread 가 spin 하므로
    여기서는 future 완료만 polling 으로 대기한다."""
    if cli is None or not cli.wait_for_service(timeout_sec=2.0):
        return None
    future = cli.call_async(req)
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            return None
        time.sleep(0.05)
    return future.result()


# ── 로봇 명령 ─────────────────────────────────────────────────────────────────

def stop_robot() -> bool:
    """비상 정지. stop_mode=1 (DR_QSTOP, Stop Category 2)."""
    req = MoveStop.Request()
    req.stop_mode = 1
    resp = _call(_cli_stop, req, timeout=5.0)
    ok = bool(resp and getattr(resp, "success", False))
    fb.push_log("WARN", "Emergency stop requested", source="system")
    return ok


def move_joint(positions, vel: float = 30, acc: float = 60,
               time_sec: float = 0.0, sync: int = 0) -> bool:
    """관절 공간 이동. sync=0(동기 완료까지 대기), sync=1(비동기)."""
    req = MoveJoint.Request()
    req.pos = [float(p) for p in positions][:6]
    req.vel = float(vel)
    req.acc = float(acc)
    req.time = float(time_sec)
    req.radius = 0.0
    req.mode = 0
    req.blend_type = 0
    req.sync_type = int(sync)
    resp = _call(_cli_move_joint, req, timeout=30.0)
    ok = bool(resp and getattr(resp, "success", False))
    fb.push_log(
        "MOVE" if ok else "WARN",
        f"move_joint {positions} → {'OK' if ok else 'FAIL'}",
        source="robot",
    )
    return ok


def set_robot_mode_autonomous() -> bool:
    """robot_mode=1 (ROBOT_MODE_AUTONOMOUS)."""
    req = SetRobotMode.Request()
    req.robot_mode = 1
    resp = _call(_cli_set_mode, req, timeout=5.0)
    return bool(resp and getattr(resp, "success", False))


# ── 호환 stub ─────────────────────────────────────────────────────────────────
# coord_service(main_side) 가 Firebase 텔레메트리에 직접 publish 하고,
# fk_worker 가 TCP 를 재계산하므로 별도 모니터 루프는 불필요.
# app.py 가 startup() 에서 호출하던 인터페이스는 유지(no-op).

def start_state_monitor():
    pass


def stop_state_monitor():
    pass
