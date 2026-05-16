"""
ros2_move_recoder.gripper_worker — OnRobot RG2/RG6 Modbus TCP 그리퍼 wrapper

* Calibration_Tutorial/onrobot.py 의 close/open/move/get_width API 채택
* 단위: 펌웨어는 1/10 mm (정수). 사용자/UI/저장은 mm 로 통일 → 변환은 워커 내부에서.
* lazy connect: worker thread 가 명령 처리 + 1Hz width polling + reconnect.

⚠ 중요한 thread-safety 정책:
  pymodbus 2.5.x 의 ModbusTcpClient (sync) 는 socket 호출이 thread-safe 하지 않다.
  여러 thread 에서 read/write 를 동시에 하면 socket 상태 corrupt → 프로세스 segfault
  (관측 사례: dsr_controller2 의 get_robot_mode_cb 와 race).
  → 모든 Modbus IO 는 단일 worker thread (`_run`) 가 단독 수행.
  → 외부 (메인) thread 의 close/open/move 슬롯은 thread-safe `queue.Queue` 에 명령
     push 만 하고 즉시 return.
"""

import os
import queue
import threading
import time

from PyQt5 import QtCore


# 환경변수로 IP/port override 가능
GRIPPER_IP   = os.environ.get("GRIPPER_IP",   "192.168.1.1")
GRIPPER_PORT = int(os.environ.get("GRIPPER_PORT", "502"))
GRIPPER_TYPE = os.environ.get("GRIPPER_TYPE", "rg2")  # rg2 / rg6
WIDTH_POLL_INTERVAL_S = 1.0    # worker thread 가 width read 주기
RECONNECT_INTERVAL_S  = 5.0    # 미연결 시 재시도 간격
CMD_QUEUE_MAX = 10
CLOSE_WIDTH_TENTH_MM = 200     # close 시 명령할 width (1/10 mm) — 20mm


class GripperWorker(QtCore.QObject):
    """OnRobot RG2/RG6 그리퍼 — 단일 thread Modbus IO.

    슬롯 (Qt thread 안전, 큐에 push 만):
      - start() / stop()
      - open() / close() / move(width_mm: float)
    시그널:
      - connected_changed(bool, str)
      - width_changed(float)
      - log(str)
    """

    connected_changed = QtCore.pyqtSignal(bool, str)
    width_changed     = QtCore.pyqtSignal(float)
    log               = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._rg = None
        self._stopped = True
        self._thread: threading.Thread | None = None
        self._connected = False
        self._last_connect_attempt = 0.0
        self._last_width: float | None = None
        self._last_cmd_state: str = "unknown"   # 'open' / 'closed' / 'mid' / 'unknown'
        # thread-safe 명령 큐 — 외부 thread 가 push, worker thread 가 pop
        self._cmd_queue: "queue.Queue[tuple[str, object]]" = \
            queue.Queue(maxsize=CMD_QUEUE_MAX)

    # ─── Public API (외부 thread 슬롯 — 즉시 return) ────────────
    @QtCore.pyqtSlot()
    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stopped = False
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="gripper-worker")
        self._thread.start()

    @QtCore.pyqtSlot()
    def stop(self):
        self._stopped = True
        # worker 가 깨어나도록 dummy push
        try:
            self._cmd_queue.put_nowait(('_wake', None))
        except queue.Full:
            pass

    @QtCore.pyqtSlot()
    def open(self):
        self._enqueue(('open', None))

    @QtCore.pyqtSlot()
    def close(self):
        # close = 20mm (CLOSE_WIDTH_TENTH_MM = 200 → 20mm) — 손/물체 보호 default
        self._enqueue(('move', CLOSE_WIDTH_TENTH_MM))

    @QtCore.pyqtSlot(float)
    def move(self, width_mm: float):
        # 펌웨어 1/10 mm 정수
        w_int = int(round(max(0.0, float(width_mm)) * 10.0))
        self._enqueue(('move', w_int))

    # ─── 외부 query (atomic read — lock 불필요) ─────────────────
    def is_connected(self) -> bool:
        return self._connected

    def last_width(self) -> float | None:
        return self._last_width

    def last_state(self) -> str:
        return self._last_cmd_state

    # ─── 내부 ─────────────────────────────────────────────────
    def _enqueue(self, cmd):
        try:
            self._cmd_queue.put_nowait(cmd)
        except queue.Full:
            self.log.emit(f"[grip] 명령 큐 full — '{cmd[0]}' 무시")

    def _ensure_rg(self) -> bool:
        """worker thread 전용. 이미 연결돼있으면 True. 안되면 connect 시도."""
        if self._rg is not None and self._connected:
            return True
        try:
            from .onrobot import RG
        except ImportError:
            try:
                from ros2_move_recoder.onrobot import RG
            except Exception as e:
                self.log.emit(f"[grip] onrobot 모듈 import 실패: {e}")
                return False
        try:
            self._rg = RG(GRIPPER_TYPE, GRIPPER_IP, GRIPPER_PORT)
            self._connected = True
            self.connected_changed.emit(
                True, f"{GRIPPER_TYPE.upper()} @ {GRIPPER_IP}:{GRIPPER_PORT}")
            self.log.emit(
                f"[grip] 연결됨 — {GRIPPER_TYPE.upper()} "
                f"(max_width={self._rg.max_width/10:.0f}mm)")
            return True
        except Exception as e:
            self._rg = None
            self._connected = False
            return False

    def _safe_disconnect(self):
        if self._rg is not None:
            try:
                self._rg.close_connection()
            except Exception:
                pass
            self._rg = None

    def _do_command(self, kind: str, arg):
        """worker thread 내부 실행. 단일 thread 라 lock 불필요."""
        if not self._ensure_rg():
            self.log.emit(
                f"[grip] '{kind}' 거부 — 미연결 ({GRIPPER_IP}:{GRIPPER_PORT})")
            return
        try:
            if kind == 'open':
                self._rg.open_gripper()
                self._last_cmd_state = 'open'
                self.log.emit("[grip] open")
            elif kind == 'move':
                w_int = min(int(arg), int(self._rg.max_width))
                w_int = max(0, w_int)
                self._rg.move_gripper(w_int)
                # state 분류: ≤20mm 닫힘, ≥(max−10mm) 열림, 그 외 중간
                if w_int <= CLOSE_WIDTH_TENTH_MM:
                    self._last_cmd_state = 'closed'
                elif w_int >= int(self._rg.max_width) - 100:
                    self._last_cmd_state = 'open'
                else:
                    self._last_cmd_state = 'mid'
                self.log.emit(
                    f"[grip] move → {w_int/10.0:.1f}mm "
                    f"(state={self._last_cmd_state})")
        except Exception as e:
            self.log.emit(f"[grip] 명령 실패: {type(e).__name__}: {e}")
            self._connected = False
            self._safe_disconnect()
            self.connected_changed.emit(False, f"명령 실패: {e}")

    def _do_width_poll(self):
        """worker thread 내부 — 1Hz width read."""
        if not self._connected or self._rg is None:
            return
        try:
            w = self._rg.get_width() / 10.0
            if self._last_width is None or \
               abs(w - (self._last_width or 0.0)) >= 0.3:
                self._last_width = w
                self.width_changed.emit(float(w))
        except Exception as e:
            self.log.emit(f"[grip] width 읽기 실패: {e}")
            self._connected = False
            self._safe_disconnect()
            self.connected_changed.emit(False, f"통신 끊김: {e}")

    def _run(self):
        """단일 thread loop — 명령 큐 처리 + 1Hz width polling + reconnect."""
        self.log.emit(
            f"[grip] worker 시작 (target {GRIPPER_IP}:{GRIPPER_PORT})")
        # 첫 connect 시도
        self._ensure_rg()
        if not self._connected:
            self.log.emit(
                f"[grip] 초기 연결 실패 — {RECONNECT_INTERVAL_S:.0f}s 마다 재시도")

        last_poll_t = time.monotonic()

        while not self._stopped:
            # 1) 명령 큐 처리 (timeout 250ms 으로 idle 시 polling 도 진행)
            try:
                cmd = self._cmd_queue.get(timeout=0.25)
            except queue.Empty:
                cmd = None

            if cmd is not None and cmd[0] != '_wake':
                self._do_command(*cmd)
                # 명령 직후 width 즉시 read (모터 진행 중일 수 있어 대략값)
                self._do_width_poll()
                last_poll_t = time.monotonic()
                continue

            now = time.monotonic()
            # 2) 주기 width polling (1초)
            if now - last_poll_t >= WIDTH_POLL_INTERVAL_S:
                last_poll_t = now
                if not self._connected:
                    if now - self._last_connect_attempt >= RECONNECT_INTERVAL_S:
                        self._last_connect_attempt = now
                        self._ensure_rg()
                else:
                    self._do_width_poll()

        # 종료 정리
        self._safe_disconnect()
        if self._connected:
            self._connected = False
            self.connected_changed.emit(False, "")
        self.log.emit("[grip] worker 종료")
