"""
ros2_move_recoder.dualsense_worker — DualSense 컨트롤러 입력을 Qt 시그널로 변환

* pygame.joystick (SDL2) 기반 — `/dev/input/js0` 사용, 별도 권한 불필요
* 별도 polling thread 가 60Hz 로 입력 읽고 Qt 시그널로 메인 스레드에 dispatch
* 연결/분리 자동 감지 + 안전 정지 (분리 시 즉시 stop_jog 발사)

매핑 (단계 2/3):
  • 좌측 스틱 ↑/→    : 다음 joint (+1, wrap) — LX/LY 중 큰 쪽, edge detect
  • 좌측 스틱 ↓/←    : 이전 joint (−1)
  • 우측 아날로그 스틱: 현재 선택된 joint 의 jog (응답 곡선 |s|^1.5)
  • D-Pad           : (예약, 미사용)
"""

import os
import threading
import time

import pygame
from PyQt5 import QtCore


# ── DualSense joydev 축 매핑 (hid-playstation 드라이버 + SDL 2.0.20) ──
# 환경에 따라 다를 수 있으면 GUI 로그의 "axes/buttons" 출력으로 검증.
AXIS_LX = 0
AXIS_LY = 1   # 위쪽이 -1
AXIS_L2 = 2   # 안눌림 -1 → 풀눌림 +1 (트리거)
AXIS_RX = 3   # SDL 2.0.20 + hid-playstation 표준 매핑
AXIS_RY = 4
AXIS_R2 = 5

# joydev hid-playstation 표준 button 인덱스 (값이 안 맞으면 GUI 로그의
# "[ds] btn N ↓" 출력으로 실제 인덱스 확인 후 아래 상수 조정)
BTN_CROSS    = 0   # × — pause/resume 토글, long-press(≥1s) → E-Stop
BTN_CIRCLE   = 1   # ○ — Smooth 자동 → Play (smooth.json 있으면 바로 Play)
BTN_TRIANGLE = 2   # △ — Home 복귀
BTN_SQUARE   = 3   # □ — 그리퍼 Open ↔ Close 토글
BTN_CREATE   = 8   # Share / Create — short=Record toggle, long(≥2s)=새 프로파일
BTN_OPTIONS  = 9   # Options (터치패드 우측) — Joint ↔ TCP jog 모드 토글
BTN_L3       = 11  # 좌측 스틱 클릭 — L3+R3 동시면 MANUAL/AUTO 모드 전환
BTN_R3       = 12  # 우측 스틱 클릭

# Jog 모드
JOG_MODE_JOINT = 'joint'
JOG_MODE_TCP   = 'tcp'

# DSR jog API 의 task axis (TCP) — ref=DR_BASE 사용
TCP_AXIS_X = 6
TCP_AXIS_Y = 7
TCP_AXIS_Z = 8
TCP_MAX_VEL_MM_S = 50.0   # X/Y/Z jog 최대 속도

# long-press 임계 (초)
CROSS_LONGPRESS_S  = 1.0   # × hold ≥ 1s → E-Stop
CREATE_LONGPRESS_S = 2.0   # Create hold ≥ 2s → 새 프로파일 자동 생성

# L2/R2 트리거 step 발동 (hold 시 자동 반복)
TRIGGER_FIRE_TH    = 0.5   # 이상이면 발동 (hold 중이면 REPEAT 주기로 반복)
TRIGGER_RESET_TH   = 0.2   # 이하로 풀리면 다음 첫 발동은 즉시
TRIGGER_REPEAT_S   = 0.10  # hold 중 자동 반복 주기 (10Hz → 1°/s × 10 = 10°/s/s)
SPEED_STEP_DEG_S   = 1.0   # 1회 당 mini-jog 속도 ± 변화량

DEADZONE = 0.15           # 우측 스틱 jog deadzone (radial)
POLL_HZ  = 60             # 입력 폴링 주기
RESCAN_INTERVAL_S = 1.0   # 미연결 시 재스캔 간격

# 좌측 스틱 LX 의 joint 선택 임계 (히스테리시스)
LSTICK_SELECT_FIRE_TH  = 0.6   # 이상 → 1회 step
LSTICK_SELECT_RESET_TH = 0.3   # 이하로 풀려야 다음 step 가능

# jog 안정화 — DSR jog() 는 1 제어 주기(10ms) 분량만 실행 후 정지.
# 연속 이동은 _JogDispatcher 가 10ms 마다 같은 vel_q 를 재발사해 보장.
# 핵심 정책:
#   1) 양자화 10°/s + change 임계 10°/s — jitter 로 인한 vel_q 미세 변동 emit 차단.
#      VEL_QUANT=5 로 낮추면 5↔10°/s jitter emit → dispatcher 가 매번 다른 vel 발사 → 끊김.
#   2) cooldown 100ms — 같은 axis 재emit 최소 간격 (빠른 vel 추적 + jitter 억제 균형)
#   3) 부호/axis 변경·정지는 즉시 emit (반응성)
#   4) 연속 이동 중 재발사는 _JogDispatcher 의 JOG_REFIRE_S(10ms) 가 담당.
VEL_QUANT_DEG_S          = 10.0   # vel 양자화 단위
VEL_CHANGE_TH_DEG_S      = 10.0   # 동일 axis/부호 시 이 미만 변화 무시
JOG_EMIT_MIN_INTERVAL_S  = 0.10   # 같은 axis 재emit 최소 간격 (100ms)


class DualSenseWorker(QtCore.QObject):
    """DualSense 입력 polling worker. start() / stop() 으로 thread 제어."""

    connected_changed     = QtCore.pyqtSignal(bool, str)        # (ok, name)
    request_jog           = QtCore.pyqtSignal(int, int, float)  # (axis 0-5, ref=0, vel)
    request_stop_jog      = QtCore.pyqtSignal()
    joint_selection_moved = QtCore.pyqtSignal(int)              # 새 선택 joint 0~5
    record_toggle         = QtCore.pyqtSignal()                 # Create short — record on/off
    new_profile_request   = QtCore.pyqtSignal()                 # Create long(≥2s) — 새 프로파일 자동 생성
    smooth_play_combo     = QtCore.pyqtSignal()                 # ○ — Smooth 후 자동 Play
    pause_resume_toggle   = QtCore.pyqtSignal()                 # × short-press
    emergency_stop        = QtCore.pyqtSignal()                 # × long-press (≥1s)
    home_request          = QtCore.pyqtSignal()                 # △ — Home
    mode_toggle_request   = QtCore.pyqtSignal()                 # L3+R3 — MANUAL ↔ AUTONOMOUS
    speed_step            = QtCore.pyqtSignal(float)            # ±SPEED_STEP_DEG_S (L2=−, R2=+)
    gripper_toggle        = QtCore.pyqtSignal()                 # □ — 그리퍼 open/close 토글
    jog_mode_changed      = QtCore.pyqtSignal(str)              # Options — 'joint'/'tcp' 토글
    log                   = QtCore.pyqtSignal(str)

    def __init__(self, jog_max_vel: float = 60.0):
        super().__init__()
        self._stopped = True
        self._thread: threading.Thread | None = None
        self._joystick = None
        self._connected = False
        self._selected_joint = 0
        self._jog_max_vel = float(jog_max_vel)
        # ⚠ jog dispatcher 직접 참조 — 메인 thread 시그널 경유 latency 제거.
        #   None 이면 fallback 으로 request_jog/request_stop_jog 시그널 emit.
        self._jog_dispatcher = None
        # latest-wins jog 캐시 — 동일값이면 재emit 안 함 (DSR dispatcher 와 이중 안전)
        self._last_jog_target: tuple[int, float] | None = None
        # × 버튼 long-press 추적
        self._cross_down_t: float | None = None
        self._cross_long_fired: bool = False
        # Create 버튼 long-press 추적
        self._create_down_t: float | None = None
        self._create_long_fired: bool = False
        # L3+R3 콤보 edge 추적 (둘 다 풀려야 다음 발사)
        self._mode_combo_armed: bool = True
        # L2/R2 트리거 hold auto-repeat — 다음 발동 가능 시각 (0=즉시)
        self._l2_next_fire_t: float = 0.0
        self._r2_next_fire_t: float = 0.0
        # ── 디버깅 ──
        self._verbose: bool = False         # 자세한 입력 로그 (메뉴 토글)
        self._poll_count: int = 0           # 누적 폴링 횟수
        self._last_health_t: float = 0.0    # 헬스 통계 마지막 출력 시각
        # 마지막 raw 값 (verbose 변화 감지용)
        self._last_rx: float = 0.0
        self._last_ry: float = 0.0
        # jog emit cooldown (같은 axis 재emit 사이 최소 간격)
        self._last_jog_emit_t: float = 0.0
        # modal 다이얼로그 활성 — True 면 jog/joint/속도 input 차단 (button 만 처리)
        self._modal_active: bool = False
        self._last_l2: float = 0.0
        self._last_r2: float = 0.0
        self._last_hat: tuple = (0, 0)
        # 좌측 스틱 LX joint 선택 히스테리시스
        self._lstick_select_armed: bool = True
        # Jog 모드 — 'joint' (J1~J6) / 'tcp' (X/Y/Z)
        self._jog_mode: str = JOG_MODE_JOINT
        self._last_l3: int = 0
        self._last_r3: int = 0

    # ─── 외부 슬롯 ─────────────────────────────────────
    @QtCore.pyqtSlot(int)
    def set_selected_joint(self, idx: int):
        if 0 <= idx < 6:
            self._selected_joint = idx

    @QtCore.pyqtSlot(float)
    def set_jog_max_vel(self, vel: float):
        self._jog_max_vel = max(1.0, float(vel))

    def set_jog_dispatcher(self, dispatcher):
        """JogDispatcher 직접 참조. 메인 thread 시그널 경유 없이 daemon thread
        에서 dispatcher.set/stop 호출 → latency 즉시 ~0."""
        self._jog_dispatcher = dispatcher
        self.log.emit("[ds] jog dispatcher 직접 연결 — 시그널 경유 우회")

    @QtCore.pyqtSlot(bool)
    def set_modal_active(self, active: bool):
        """GUI 가 modal 다이얼로그 활성 시 통보. True 면 jog/joint/속도 input 차단.
        button 이벤트는 모달 응답을 위해 그대로 dispatch."""
        was = self._modal_active
        self._modal_active = bool(active)
        if was and not active:
            self.log.emit("[ds] modal 닫힘 — jog 재개")
        elif not was and active:
            self.log.emit("[ds] modal 활성 — jog/joint/속도 일시정지")
            # 즉시 안전 정지 — dispatcher 직접 호출
            if self._last_jog_target is not None:
                if self._jog_dispatcher is not None:
                    self._jog_dispatcher.stop()
                else:
                    self._post_main(self.request_stop_jog.emit)
                self._last_jog_target = None

    @QtCore.pyqtSlot(bool)
    def set_verbose(self, on: bool):
        self._verbose = bool(on)
        self.log.emit(f"[ds][debug] verbose 모드 {'ON' if on else 'OFF'}")
        if on and self._joystick is not None:
            j = self._joystick
            try:
                self.log.emit(
                    f"[ds][debug] capabilities — "
                    f"axes={j.get_numaxes()}, "
                    f"buttons={j.get_numbuttons()}, "
                    f"hats={j.get_numhats()}, "
                    f"name={j.get_name()!r}, "
                    f"id={j.get_instance_id()}, "
                    f"guid={j.get_guid()}")
            except Exception as e:
                self.log.emit(f"[ds][debug] capabilities 조회 실패: {e}")

    def _log_v(self, msg: str):
        if self._verbose:
            self.log.emit(msg)

    def _post_main(self, callable_):
        """daemon thread 에서 시그널 emit 호출 — Qt 의 cross-thread queued
        connection 이 자동으로 메인 thread 에 dispatch."""
        # ⚠ 이전에 QTimer.singleShot(0, ...) 으로 wrap 했으나, singleShot 은 caller
        #   thread 에 event loop 를 요구하는데 daemon Python thread 엔 없음 →
        #   timer 가 fire 안 되는 문제가 있어서 직접 호출로 복귀.
        callable_()

    # ─── 워커 lifecycle ───────────────────────────────
    @QtCore.pyqtSlot()
    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stopped = False
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="dualsense-worker")
        self._thread.start()

    @QtCore.pyqtSlot()
    def stop(self):
        self._stopped = True
        # 마지막 안전 정지
        if self._last_jog_target is not None:
            self.request_stop_jog.emit()
            self._last_jog_target = None

    # ─── 내부 ─────────────────────────────────────────
    def _ensure_pygame(self):
        # 헤드리스 환경/Wayland 안전 — video subsystem 미초기화
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        if not pygame.get_init():
            pygame.init()
        if not pygame.joystick.get_init():
            pygame.joystick.init()

    def _try_open(self) -> bool:
        try:
            self._ensure_pygame()
        except Exception as e:
            self.log.emit(f"[ds] pygame init 실패: {e}")
            return False
        # 핫플러그 감지 위해 quit/init 토글
        pygame.joystick.quit()
        pygame.joystick.init()
        for i in range(pygame.joystick.get_count()):
            try:
                j = pygame.joystick.Joystick(i)
                j.init()
                name = j.get_name() or "?"
                if any(k in name for k in
                       ("DualSense", "Wireless Controller", "Sony")):
                    self._joystick = j
                    self._connected = True
                    self.connected_changed.emit(True, name)
                    self.log.emit(
                        f"[ds] 연결됨 — {name}  "
                        f"(axes={j.get_numaxes()}, "
                        f"buttons={j.get_numbuttons()}, "
                        f"hats={j.get_numhats()})")
                    return True
            except Exception:
                continue
        return False

    def _close(self, log_msg: str | None = None):
        # 안전 정지 먼저
        if self._last_jog_target is not None:
            self.request_stop_jog.emit()
            self._last_jog_target = None
        if self._joystick is not None:
            try:
                self._joystick.quit()
            except Exception:
                pass
            self._joystick = None
        if self._connected:
            self._connected = False
            self.connected_changed.emit(False, "")
            if log_msg:
                self.log.emit(log_msg)

    def _run(self):
        self.log.emit("[ds] 폴링 시작 (60Hz)")
        period = 1.0 / POLL_HZ
        last_hat = (0, 0)
        last_rescan = 0.0
        while not self._stopped:
            t0 = time.monotonic()
            try:
                if not self._connected:
                    if t0 - last_rescan >= RESCAN_INTERVAL_S:
                        last_rescan = t0
                        if self._try_open():
                            last_hat = (0, 0)
                    if not self._connected:
                        time.sleep(0.1)
                        continue

                # 이벤트 큐 처리 — button edge 시점 1회만 emit
                # (event.get() 이 pump 까지 자동 수행)
                now = time.monotonic()
                self._poll_count += 1
                for ev in pygame.event.get():
                    if ev.type == pygame.JOYBUTTONDOWN:
                        # 인덱스 진단 — verbose 모드 (Ctrl+Shift+D) 일 때만
                        self._log_v(f"[ds] btn {ev.button} ↓")
                        if ev.button == BTN_CIRCLE:
                            self._post_main(self.smooth_play_combo.emit)
                        elif ev.button == BTN_TRIANGLE:
                            self._post_main(self.home_request.emit)
                        elif ev.button == BTN_SQUARE:
                            self._post_main(self.gripper_toggle.emit)
                        elif ev.button == BTN_OPTIONS:
                            # Joint ↔ TCP 모드 토글
                            new_mode = (JOG_MODE_TCP if self._jog_mode == JOG_MODE_JOINT
                                        else JOG_MODE_JOINT)
                            self._jog_mode = new_mode
                            # 모드 전환 시 잔여 jog 즉시 정지
                            if self._jog_dispatcher is not None and \
                               self._last_jog_target is not None:
                                self._jog_dispatcher.stop()
                            elif self._last_jog_target is not None:
                                self._post_main(self.request_stop_jog.emit)
                            self._last_jog_target = None
                            self._last_jog_emit_t = 0.0
                            self.log.emit(f"[ds] jog mode → {new_mode.upper()}")
                            self._post_main(
                                lambda m=new_mode: self.jog_mode_changed.emit(m))
                        elif ev.button == BTN_CROSS:
                            self._cross_down_t = now
                            self._cross_long_fired = False
                        elif ev.button == BTN_CREATE:
                            self._create_down_t = now
                            self._create_long_fired = False
                    elif ev.type == pygame.JOYBUTTONUP:
                        self._log_v(f"[ds][debug] btn {ev.button} ↑")
                        if ev.button == BTN_CROSS:
                            if self._cross_down_t is not None and \
                               not self._cross_long_fired:
                                self._post_main(self.pause_resume_toggle.emit)
                            self._cross_down_t = None
                            self._cross_long_fired = False
                        elif ev.button == BTN_CREATE:
                            if self._create_down_t is not None and \
                               not self._create_long_fired:
                                self._post_main(self.record_toggle.emit)
                            self._create_down_t = None
                            self._create_long_fired = False

                # × hold ≥ 1s → E-Stop 즉시 발사
                if self._cross_down_t is not None and \
                   not self._cross_long_fired and \
                   (now - self._cross_down_t) >= CROSS_LONGPRESS_S:
                    self._post_main(self.emergency_stop.emit)
                    self._cross_long_fired = True
                    self.log.emit(
                        f"[ds] × long-press ({CROSS_LONGPRESS_S:.1f}s) → E-STOP")

                # Create hold ≥ 2s → 새 프로파일 자동 생성
                if self._create_down_t is not None and \
                   not self._create_long_fired and \
                   (now - self._create_down_t) >= CREATE_LONGPRESS_S:
                    self._post_main(self.new_profile_request.emit)
                    self._create_long_fired = True
                    self.log.emit(
                        f"[ds] Create long-press ({CREATE_LONGPRESS_S:.1f}s) → 새 프로파일")

                j = self._joystick

                # 연결 살아있는지 빠른 확인
                try:
                    nax = j.get_numaxes()
                except Exception as e:
                    self._close(f"[ds] 연결 끊김: {e}")
                    continue

                # ── modal 떠있으면 입력 차단 (button event 만 위에서 처리됨) ──
                if self._modal_active:
                    elapsed = time.monotonic() - t0
                    if elapsed < period:
                        time.sleep(period - elapsed)
                    continue

                # ── 좌측 스틱 raw 읽기 (mode 분기 전 공통) ──
                lx_raw_l = j.get_axis(AXIS_LX) if nax > AXIS_LX else 0.0
                ly_raw_l = -j.get_axis(AXIS_LY) if nax > AXIS_LY else 0.0  # 위가 +
                # 트리거 매핑 의심 안전장치
                if abs(lx_raw_l) > 0.95 and self._poll_count < 30:
                    if not getattr(self, "_warned_lx_select", False):
                        self.log.emit(
                            f"[ds] ⚠ AXIS_LX={AXIS_LX} idle {lx_raw_l:+.2f} "
                            f"→ 트리거 매핑 의심")
                        self._warned_lx_select = True
                    lx_raw_l = 0.0
                if abs(ly_raw_l) > 0.95 and self._poll_count < 30:
                    if not getattr(self, "_warned_ly_select", False):
                        self.log.emit(
                            f"[ds] ⚠ AXIS_LY={AXIS_LY} idle {-ly_raw_l:+.2f} "
                            f"→ 트리거 매핑 의심")
                        self._warned_ly_select = True
                    ly_raw_l = 0.0

                # ── JOINT 모드: 좌측 스틱 → joint 선택 (히스테리시스 edge) ──
                # ── TCP 모드: 좌측 스틱은 jog 입력 (X=LX, Y=LY) → 아래 통합 처리 ──
                if self._jog_mode == JOG_MODE_JOINT:
                    lstick_sel = (ly_raw_l if abs(ly_raw_l) > abs(lx_raw_l)
                                  else lx_raw_l)
                    if abs(lstick_sel) >= LSTICK_SELECT_FIRE_TH \
                       and self._lstick_select_armed:
                        delta = +1 if lstick_sel > 0 else -1
                        new_idx = (self._selected_joint + delta) % 6
                        if self._jog_dispatcher is not None and \
                           self._last_jog_target is not None:
                            self._jog_dispatcher.stop()
                        elif self._last_jog_target is not None:
                            self._post_main(self.request_stop_jog.emit)
                        self._last_jog_target = None
                        self._last_jog_emit_t = 0.0
                        self._selected_joint = new_idx
                        self._lstick_select_armed = False
                        self._post_main(
                            lambda i=new_idx:
                            self.joint_selection_moved.emit(i))
                    elif abs(lstick_sel) <= LSTICK_SELECT_RESET_TH:
                        self._lstick_select_armed = True

                # D-Pad: verbose 로그용으로만 추적 (joint 선택은 좌측 스틱)
                if j.get_numhats() > 0:
                    cur_hat = j.get_hat(0)
                    if cur_hat != self._last_hat:
                        self._log_v(
                            f"[ds][debug] hat {self._last_hat} → {cur_hat}")
                        self._last_hat = cur_hat

                # ── L3+R3 동시 누름 → MANUAL ↔ AUTONOMOUS 토글 ──
                # 콤보 발사 후엔 둘 중 하나라도 풀어야 다음 발사 가능
                try:
                    nbtn = j.get_numbuttons()
                    l3 = j.get_button(BTN_L3) if nbtn > BTN_L3 else 0
                    r3 = j.get_button(BTN_R3) if nbtn > BTN_R3 else 0
                except Exception:
                    l3 = r3 = 0
                if (l3, r3) != (self._last_l3, self._last_r3):
                    self._log_v(
                        f"[ds][debug] L3={l3} R3={r3} "
                        f"(armed={self._mode_combo_armed})")
                    self._last_l3, self._last_r3 = l3, r3
                if l3 and r3:
                    if self._mode_combo_armed:
                        self._post_main(self.mode_toggle_request.emit)
                        self._mode_combo_armed = False
                        self.log.emit("[ds] L3+R3 → 모드 전환")
                else:
                    self._mode_combo_armed = True

                # ── L2/R2 트리거 → mini-jog 속도 step (히스테리시스) ──
                # 트리거 axis: 안눌림 -1, 풀눌림 +1 → 0~1 정규화
                l2 = (j.get_axis(AXIS_L2) + 1.0) * 0.5 if nax > AXIS_L2 else 0.0
                r2 = (j.get_axis(AXIS_R2) + 1.0) * 0.5 if nax > AXIS_R2 else 0.0
                if abs(l2 - self._last_l2) >= 0.05:
                    self._log_v(
                        f"[ds][debug] L2 {self._last_l2:.2f} → {l2:.2f}")
                    self._last_l2 = l2
                if abs(r2 - self._last_r2) >= 0.05:
                    self._log_v(
                        f"[ds][debug] R2 {self._last_r2:.2f} → {r2:.2f}")
                    self._last_r2 = r2
                # L2: hold 중 REPEAT 주기로 −SPEED_STEP_DEG_S 반복 발사
                if l2 >= TRIGGER_FIRE_TH:
                    if now >= self._l2_next_fire_t:
                        self._post_main(
                            lambda d=-SPEED_STEP_DEG_S: self.speed_step.emit(d))
                        self._l2_next_fire_t = now + TRIGGER_REPEAT_S
                elif l2 <= TRIGGER_RESET_TH:
                    self._l2_next_fire_t = 0.0
                if r2 >= TRIGGER_FIRE_TH:
                    if now >= self._r2_next_fire_t:
                        self._post_main(
                            lambda d=+SPEED_STEP_DEG_S: self.speed_step.emit(d))
                        self._r2_next_fire_t = now + TRIGGER_REPEAT_S
                elif r2 <= TRIGGER_RESET_TH:
                    self._r2_next_fire_t = 0.0

                # ── 우측 스틱 → 선택 joint jog (RX/RY 중 큰 쪽) ──
                # ⚠ 안전장치: 트리거 axis 가 안눌림 시 -1 을 반환하므로,
                #   매핑 잘못된 환경에서 jog 가 자동 발사되는 것을 방지하기 위해
                #   첫 폴링에서 |raw| > 0.95 인 axis 는 트리거로 간주하고 0 처리.
                #   (정상 스틱은 중앙에서 ~0, 풀로 밀어도 ±1 까지 인 정상 범위)
                _rx_raw = j.get_axis(AXIS_RX) if nax > AXIS_RX else 0.0
                _ry_raw = j.get_axis(AXIS_RY) if nax > AXIS_RY else 0.0
                if abs(_rx_raw) > 0.95 and self._poll_count < 30:
                    # 첫 0.5초 동안 ±1 부근이면 트리거로 추정 → 무시 + 1회 경고
                    if not getattr(self, "_warned_rx", False):
                        self.log.emit(
                            f"[ds] ⚠ AXIS_RX={AXIS_RX} 의 idle 값이 "
                            f"{_rx_raw:+.2f} → 트리거 매핑 의심. "
                            f"디버그 모드(Ctrl+Shift+D)로 axis 매핑 확인 권장")
                        self._warned_rx = True
                    _rx_raw = 0.0
                if abs(_ry_raw) > 0.95 and self._poll_count < 30:
                    if not getattr(self, "_warned_ry", False):
                        self.log.emit(
                            f"[ds] ⚠ AXIS_RY={AXIS_RY} 의 idle 값이 "
                            f"{_ry_raw:+.2f} → 트리거 매핑 의심")
                        self._warned_ry = True
                    _ry_raw = 0.0
                rx = _rx_raw
                ry = -_ry_raw  # 위가 +
                if abs(rx - self._last_rx) >= 0.05:
                    self._log_v(
                        f"[ds][debug] RX {self._last_rx:+.2f} → {rx:+.2f}")
                    self._last_rx = rx
                if abs(ry - self._last_ry) >= 0.05:
                    self._log_v(
                        f"[ds][debug] RY {self._last_ry:+.2f} → {ry:+.2f} "
                        f"(joint J{self._selected_joint+1})")
                    self._last_ry = ry
                # ── jog 결정 — JOINT vs TCP 모드 분기 ──
                # JOINT 모드: 우측 스틱 (RX/RY 중 큰 쪽) → 선택 joint
                # TCP 모드: 좌측 LX → X, 좌측 LY → Y, 우측 RY → Z (가장 큰 절댓값)
                if self._jog_mode == JOG_MODE_JOINT:
                    use = ry if abs(ry) > abs(rx) else rx
                    sel_axis = self._selected_joint   # 0~5
                    sel_max_vel = self._jog_max_vel    # °/s
                else:  # TCP
                    candidates = [
                        (TCP_AXIS_X, lx_raw_l),
                        (TCP_AXIS_Y, ly_raw_l),
                        (TCP_AXIS_Z, ry),   # ry: 위가 +
                    ]
                    sel_axis, use = max(candidates, key=lambda c: abs(c[1]))
                    sel_max_vel = TCP_MAX_VEL_MM_S    # mm/s
                if abs(use) < DEADZONE:
                    if self._last_jog_target is not None:
                        if self._jog_dispatcher is not None:
                            self._jog_dispatcher.stop()
                        else:
                            self._post_main(self.request_stop_jog.emit)
                        self._last_jog_target = None
                        self._last_jog_emit_t = now
                        self._log_v(
                            f"[ds][debug] jog stop (use={use:+.2f})")
                else:
                    sign = 1.0 if use > 0 else -1.0
                    vel = sign * (abs(use) ** 1.2) * sel_max_vel
                    vel_q = round(vel / VEL_QUANT_DEG_S) * VEL_QUANT_DEG_S
                    target = (sel_axis, vel_q)
                    last = self._last_jog_target

                    immediate = (
                        last is None
                        or last[0] != sel_axis
                        or (last[1] >= 0) != (vel_q >= 0)
                    )
                    if immediate:
                        should_emit = True
                    else:
                        cooldown_ok = (now - self._last_jog_emit_t) \
                                      >= JOG_EMIT_MIN_INTERVAL_S
                        change_ok   = abs(last[1] - vel_q) >= VEL_CHANGE_TH_DEG_S
                        should_emit = cooldown_ok and change_ok

                    if should_emit:
                        ax, vq = sel_axis, float(vel_q)
                        # dispatcher 직접 호출 — 시그널 경유 우회 (latency ~0)
                        if self._jog_dispatcher is not None:
                            self._jog_dispatcher.set(ax, 0, vq)
                        else:
                            self._post_main(
                                lambda a=ax, v=vq:
                                self.request_jog.emit(a, 0, v))
                        self._last_jog_target = target
                        self._last_jog_emit_t = now
                        self._log_v(
                            f"[ds][debug] jog J{ax+1} vel={vq:+.0f}°/s "
                            f"(raw use={use:+.2f})")

            except Exception as e:
                import traceback
                self._close(f"[ds] 폴링 예외: {type(e).__name__}: {e}")
                if self._verbose:
                    for line in traceback.format_exc().strip().split("\n")[-4:]:
                        self.log.emit(f"[ds][debug]   {line}")

            elapsed = time.monotonic() - t0
            # 폴링 cycle 이 너무 느리면 워닝 (verbose 시만 — 시끄러움 방지)
            if elapsed > period * 1.5:
                self._log_v(
                    f"[ds][debug] 폴링 지연: {elapsed*1000:.1f}ms "
                    f"(목표 {period*1000:.1f}ms)")
            # 5초마다 헬스 통계 (verbose 시만)
            t_now = time.monotonic()
            if self._last_health_t == 0.0:
                self._last_health_t = t_now
                self._poll_count = 0
            elif self._verbose and (t_now - self._last_health_t) >= 5.0:
                dt = t_now - self._last_health_t
                hz = self._poll_count / dt
                self.log.emit(
                    f"[ds][debug] 헬스 — poll {hz:.1f} Hz "
                    f"(누적 {self._poll_count}/{dt:.1f}s), "
                    f"connected={self._connected}, "
                    f"selected J{self._selected_joint+1}, "
                    f"max_vel={self._jog_max_vel:.1f} °/s")
                self._poll_count = 0
                self._last_health_t = t_now
            if elapsed < period:
                time.sleep(period - elapsed)

        self._close()
        try:
            pygame.quit()
        except Exception:
            pass
        self.log.emit("[ds] 폴링 종료")
