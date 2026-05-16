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
    GRIP_MIN_GAP_S = 2.0

    def __init__(self, sm, gripper, play_vel, logger):
        self.sm = sm
        self.gripper = gripper
        self.play_vel = play_vel
        self.logger = logger
        self.measured_duration = None
        self._events = list(sm.get("gripper_events") or [])
        self._stop = threading.Event()    # abort: drop remaining events
        self._flush = threading.Event()   # normal end: emit remaining now
        self._thr = None
        self._t0 = None

    def _est_duration(self):
        sm = self.sm
        smooth_vel = float(sm.get("vel", self.play_vel) or self.play_vel)
        ratio = (smooth_vel / self.play_vel) if self.play_vel else 1.0
        md = float(sm.get("measured_duration_sec", 0.0) or 0.0)
        if md > 0:
            return md * ratio
        rd = float(sm.get("record_duration_sec", 0.0) or 0.0)
        if rd > 0:
            return rd * ratio
        return 1.0  # 최후 보정(아크길이 추정 불가시)

    def _apply(self, kind, width_mm):
        if kind == "open":
            self.gripper.open()
        elif kind == "close":
            self.gripper.close()
        else:
            self.gripper.move(width_mm)

    def _run(self):
        if not self._events:
            _log(self.logger, "[grip][play] skip — gripper_events 없음")
            return
        dur = self._est_duration()
        self._t0 = time.monotonic()
        last_emit_t = -1e9
        for ev in self._events:
            if self._stop.is_set():
                return
            target_t = float(ev.get("t_norm", 0.0)) * dur
            target_t = max(target_t, last_emit_t + self.GRIP_MIN_GAP_S)
            while not self._stop.is_set() and not self._flush.is_set():
                now = time.monotonic() - self._t0
                if now >= target_t:
                    break
                time.sleep(min(0.02, target_t - now))
            if self._stop.is_set():
                return
            self._apply(ev.get("kind"), ev.get("width_mm"))
            last_emit_t = target_t

    def start(self):
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    def stop(self):
        """abort: drop remaining events, do not force final state."""
        self._stop.set()
        if self._thr:
            self._thr.join(timeout=1.0)

    def finish(self):
        """normal end: emit any remaining events immediately, in order."""
        self._flush.set()
        if self._thr:
            self._thr.join(timeout=5.0)
        if self._t0 is not None:
            self.measured_duration = round(time.monotonic() - self._t0, 3)
