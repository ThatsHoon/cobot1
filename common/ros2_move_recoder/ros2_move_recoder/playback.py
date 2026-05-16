"""ros2_move_recoder.playback — 헤드리스 재생 코어.

PyQt/DualSense/input() 의존 없음. gui.py·player.py·robo_chef.sequence_runner 가 공유.
amovesj 비동기 모션 + smooth.json gripper_events 타임라인 재생.
"""
from __future__ import annotations
import json
import os
import threading
import time
from dataclasses import dataclass


@dataclass
class PlayResult:
    ok: bool
    duration_sec: float
    error: str | None = None
    measured_duration_sec: float | None = None


@dataclass
class _DSR:
    amovesj: object
    check_motion: object
    posj: object
    get_robot_mode: object
    ROBOT_MODE_AUTONOMOUS: object
    sstop: object


def _dsr() -> _DSR:
    """실로봇 DSR_ROBOT2 바인딩. 테스트는 monkeypatch 로 대체."""
    from DSR_ROBOT2 import (amovesj, check_motion, posj, get_robot_mode,
                            ROBOT_MODE_AUTONOMOUS)
    try:
        from DSR_ROBOT2 import stop as _stop, DR_SSTOP
        sstop = lambda: _stop(DR_SSTOP)
    except Exception:
        sstop = lambda: None
    return _DSR(amovesj, check_motion, posj, get_robot_mode,
                ROBOT_MODE_AUTONOMOUS, sstop)


def _log(logger, msg):
    (logger.info if logger else print)(msg)


def play_segment(smooth_path: str, *, gripper=None,
                  require_autonomous: bool = True, on_progress=None,
                  abort_event: "threading.Event | None" = None,
                  logger=None) -> PlayResult:
    if not os.path.isfile(smooth_path):
        return PlayResult(False, 0.0, f"missing segment file: {smooth_path}")
    with open(smooth_path) as f:
        sm = json.load(f)

    dsr = _dsr()
    if require_autonomous:
        try:
            if dsr.get_robot_mode() != dsr.ROBOT_MODE_AUTONOMOUS:
                return PlayResult(False, 0.0, "robot not in AUTONOMOUS mode")
        except Exception as e:  # noqa: BLE001
            return PlayResult(False, 0.0, f"get_robot_mode failed: {e}")

    waypoints = sm.get("waypoints_deg") or []
    vel = float(sm.get("vel", 30.0))
    acc = float(sm.get("acc", 60.0))
    if not waypoints:
        return PlayResult(False, 0.0, "no waypoints_deg in smooth.json")

    if on_progress:
        on_progress("amovesj_start")
    pts = [dsr.posj(*w) for w in waypoints]
    t0 = time.monotonic()
    rc = dsr.amovesj(pts, vel=vel, acc=acc)
    if rc != 0:
        dsr.sstop()
        return PlayResult(False, time.monotonic() - t0, f"amovesj rc={rc}")

    grip = _GripperTimeline(sm, gripper, vel, logger) if gripper else None
    if grip:
        grip.start()
    try:
        while dsr.check_motion():
            if abort_event is not None and abort_event.is_set():
                dsr.sstop()
                if grip:
                    grip.stop()
                return PlayResult(False, time.monotonic() - t0, "aborted")
            time.sleep(0.05)
    finally:
        if grip:
            grip.finish()
    dt = time.monotonic() - t0
    if on_progress:
        on_progress("done")
    return PlayResult(True, dt, None, grip.measured_duration if grip else None)


class _GripperTimeline:
    """gui.py 의 _start_gripper_play_timeline / _ensure_gripper_final_state 로직을
    self.* 의존 없이 이식. PORTING TASK: Task 2 에서 본문을 채운다."""
    def __init__(self, sm, gripper, play_vel, logger):
        self.sm = sm
        self.gripper = gripper
        self.play_vel = play_vel
        self.logger = logger
        self.measured_duration = None

    def start(self):  # pragma: no cover - Task 2 에서 구현
        raise NotImplementedError

    def stop(self):  # pragma: no cover
        raise NotImplementedError

    def finish(self):  # pragma: no cover
        raise NotImplementedError
