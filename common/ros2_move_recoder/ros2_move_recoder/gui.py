"""
ros2_move_recoder.gui — 매크로 레코더 GUI (PyQt5 + ROS 2 + DSR_ROBOT2)
================================================================
티칭펜던트로 로봇을 시연 → 기록 → 평활화 → 자동 재생을 단일 GUI 에서 처리.

기능
  * 시작 시 dsr_bringup2 launch 모드 선택 (real / virtual / skip)
  * /dsr01/joint_states 실시간 구독 — 6축 좌표 패널 + 기록 로그
  * Record / Stop / Smooth / Play 버튼
  * 좌측 사이드바: records/ 폴더 자동 스캔 → 클릭으로 액션 로드
  * 로봇 모드 자동 감지 (MANUAL / AUTONOMOUS) + 알람 클리어 / 홈 복귀
  * 평활화 파라미터 조정 (window / max_pts / vel / acc)
  * 재생 진행률 + 비상 정지 (movestop)

실행: ros2 run ros2_move_recoder gui
"""

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

# DSR — 노드 등록은 spawn 시점에 동적으로 (Multi-robot 미지원, namespace 고정 필수)
import DR_init

# DualSense 컨트롤러 워커 (pygame 기반, lazy 활성화)
from ros2_move_recoder.dualsense_worker import DualSenseWorker
# OnRobot RG 그리퍼 워커 (Modbus TCP, lazy 활성화)
from ros2_move_recoder.gripper_worker import GripperWorker
# 평활화 — single source of truth (CLI smoother 와 GUI 가 동일 함수 사용)
from ros2_move_recoder.smoother import smooth_and_save
# 재생 코어 — player.py / sequence_runner 와 공유 (gui 는 신호 기반 경로 유지,
# play_segment 는 헤드리스 경로에서만 사용. import 는 향후 일원화 대비 + 일관성).
from ros2_move_recoder.playback import play_segment  # noqa: F401

ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"

# ★★* Name mangling 회피 ★★★
# `DR_init.__dsr__node` 처럼 `__name` 패턴(앞 __, 끝 비__)은 클래스 본문 안에서
# `_ClassName__name` 으로 변형된다. 클래스 메서드 내부에서 직접 접근하면
# 엉뚱한 속성에 set/get 되어 DSR_ROBOT2 가 영원히 None 노드를 보게 됨.
# → 모듈 레벨 헬퍼로 통일해서 호출.
def _dr_set_node(node):
    setattr(DR_init, "__dsr__node", node)
def _dr_get_node():
    return getattr(DR_init, "__dsr__node", None)
def _dr_set_id(rid):
    setattr(DR_init, "__dsr__id", rid)
def _dr_set_model(model):
    setattr(DR_init, "__dsr__model", model)

_dr_set_id(ROBOT_ID)
_dr_set_model(ROBOT_MODEL)

# 경로
PKG_DIR     = Path("~/cobot_ws/src/ros2_move_recoder").expanduser()
RECORDS_DIR = PKG_DIR / "records"

# ROBOT_MODE 상수 (DSR_ROBOT2 임포트 전에는 직접 매핑)
MODE_MANUAL     = 0
MODE_AUTONOMOUS = 1
MODE_NAMES      = {0: "MANUAL", 1: "AUTONOMOUS", 2: "MEASURE"}

# 홈 자세
HOME_POSJ = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ════════════════════════════════════════════════════════════════════
# ROS 2 노드 — 단일 노드에서 sub + DSR 제어 모두 처리
# (DR_init.__dsr__node 로 등록되어 DSR_ROBOT2 가 이 노드를 사용)
# ════════════════════════════════════════════════════════════════════
class MacroNode(Node):
    """매크로 레코더 전용 ROS2 노드 — DSR_ROBOT2 의 g_node 역할 전용.
    * joint_states 구독은 별도의 JointStateNode 가 담당 (executor 분리)."""

    def __init__(self):
        super().__init__("macro_gui_node", namespace=ROBOT_ID)


# ════════════════════════════════════════════════════════════════════
# Joint State 전용 노드 — DSR g_node 와 분리하여 executor 자원 경합 회피
# ════════════════════════════════════════════════════════════════════
# * 왜 분리?
#   DSR_ROBOT2 의 service 호출이 글로벌 executor 의 spin 자원을 점유하는
#   동안 같은 executor 의 subscription dispatch 가 starve 됨 (실측 0.3Hz,
#   3초마다 +1 패턴 — 정확히 mode_timer 주기와 일치).
#   subscription 을 완전히 별도의 노드 + 별도 SingleThreadedExecutor + 별도
#   thread 에 두면 DSR service 호출과 무관하게 100Hz 처리 가능.
# ════════════════════════════════════════════════════════════════════
class JointStateNode(Node):
    """오직 /dsr01/joint_states 구독만 담당. DSR 과 무관."""

    def __init__(self, on_joint, on_debug=None):
        super().__init__("joint_state_listener", namespace=ROBOT_ID)
        self._on_joint = on_joint
        self._on_debug = on_debug or (lambda s: None)
        self._handle_count = 0
        self._success_count = 0
        self._error_count = 0
        self._last_err_t = 0.0
        self._first_msg_logged = False

        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            JointState, "/dsr01/joint_states",
            self._handle_joint, qos,
        )
        self.get_logger().info("JointStateNode 준비 완료 (별도 executor)")

    def _handle_joint(self, msg: JointState):
        self._handle_count += 1
        if not self._first_msg_logged:
            self._first_msg_logged = True
            try:
                names = list(msg.name)
                pos_deg = [round(math.degrees(float(p)), 3) for p in msg.position]
                self._on_debug(
                    f"[js] 첫 JointState 수신 — names={names}, "
                    f"len(pos)={len(msg.position)}, pos_deg={pos_deg}")
            except Exception as e:
                self._on_debug(f"[js] 첫 메시지 inspect 실패: {e}")
        try:
            name_to_pos = dict(zip(msg.name, msg.position))
            joint_deg = [round(math.degrees(float(name_to_pos[f"joint_{i}"])), 3)
                         for i in range(1, 7)]
        except (KeyError, ValueError) as e:
            self._error_count += 1
            now = time.monotonic()
            if now - self._last_err_t > 1.0:
                self._last_err_t = now
                self._on_debug(
                    f"[js] ⚠️ 좌표 추출 실패 ({type(e).__name__}: {e}) "
                    f"누적 {self._error_count}/{self._handle_count}, "
                    f"msg.name={list(msg.name)[:8]}")
            return
        self._success_count += 1
        self._on_joint(joint_deg)

    def diag_snapshot(self) -> dict:
        return {
            "handle": self._handle_count,
            "success": self._success_count,
            "error": self._error_count,
        }


# ════════════════════════════════════════════════════════════════════
# Bringup 선택 다이얼로그
# ════════════════════════════════════════════════════════════════════
class BringupDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("dsr_bringup2 모드 선택")
        self.setModal(True)
        self.setMinimumWidth(420)
        self.choice = None

        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(QtWidgets.QLabel(
            "<b>dsr_bringup2_rviz.launch.py</b> 를 어느 모드로 실행할까요?\n"
            "(이미 다른 터미널에서 실행 중이면 Skip 선택)"))

        for label, mode, accent, bg, fg in [
            ("Real    ·  192.168.1.100 : 12345", "real",
             "#dc2626", "#ffffff", "#1a1d23"),
            ("Virtual ·  시뮬레이터",            "virtual",
             "#4f46e5", "#ffffff", "#1a1d23"),
            ("Skip    ·  이미 실행 중",          "skip",
             "#9ca3af", "#ffffff", "#1a1d23"),
        ]:
            b = QtWidgets.QPushButton(label)
            b.setCursor(QtCore.Qt.PointingHandCursor)
            b.setStyleSheet(
                f"QPushButton {{"
                f"  text-align:left; padding:14px 16px; font-size:11pt;"
                f"  font-weight:600; color:{fg}; background:{bg};"
                f"  border:1px solid {accent}; border-radius:8px;"
                f"  border-left:4px solid {accent};"
                f"}}"
                f"QPushButton:hover {{ background:#f5f6f8; }}"
            )
            b.clicked.connect(lambda _, m=mode: self._select(m))
            lay.addWidget(b)

    def _select(self, mode):
        self.choice = mode
        self.accept()


# ════════════════════════════════════════════════════════════════════
# Bringup launch 관리 (별도 프로세스)
# ════════════════════════════════════════════════════════════════════
class BringupManager:
    # * virtual 시뮬레이터는 로컬에서 동작하므로 127.0.0.1 필수.
    #   real 의 IP 를 그대로 쓰면 dsr_hw_interface2 가 외부로 connect 시도 →
    #   spawner timeout 발생 + controller 활성화 실패.
    HOST_BY_MODE = {
        "real":    "192.168.1.100",
        "virtual": "127.0.0.1",
    }
    PORT = "12345"

    def __init__(self):
        self.proc: subprocess.Popen | None = None
        self.mode: str | None = None

    def launch(self, mode: str):
        if mode == "skip":
            self.mode = "external"
            return
        # 이전 proc 가 남아있으면 leak 방지를 위해 먼저 정리
        if self.proc and self.proc.poll() is None:
            self.shutdown()
        host = self.HOST_BY_MODE.get(mode, "127.0.0.1")
        cmd = [
            "ros2", "launch", "dsr_bringup2", "dsr_bringup2_rviz.launch.py",
            f"mode:={mode}",
            f"host:={host}",
            f"port:={self.PORT}",
            f"model:={ROBOT_MODEL}",
        ]
        # * stdout/stderr 를 부모 터미널로 흘림 — 컨트롤러 spawner 실패 등을 즉시 볼 수 있도록
        #   (DEVNULL 로 막으면 디버깅 불가)
        self.proc = subprocess.Popen(
            cmd, preexec_fn=os.setsid,
        )
        self.mode = mode

    def shutdown(self):
        """SIGINT → wait(5s) → SIGKILL → wait(2s, reap) 의 3-단 종료.
        마지막 wait 까지 해야 좀비(<defunct>) 가 남지 않는다."""
        if not self.proc or self.proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(self.proc.pid)
        except ProcessLookupError:
            return

        try:
            os.killpg(pgid, signal.SIGINT)
            self.proc.wait(timeout=5)
            return
        except subprocess.TimeoutExpired:
            pass
        except Exception:
            pass

        # graceful 실패 → 강제 종료 + reap
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            return
        except Exception:
            pass
        try:
            self.proc.wait(timeout=2)
        except Exception:
            pass


# ════════════════════════════════════════════════════════════════════
# DSR Worker — 모션은 모두 별도 스레드에서 (50ms 콜백 규약 + 블로킹 회피)
# ════════════════════════════════════════════════════════════════════
# ════════════════════════════════════════════════════════════════════
# Jog Dispatcher — latest-wins 워커 스레드 (manage/Manage_jog.py 포팅)
# ════════════════════════════════════════════════════════════════════
# 문제: jog 는 연속 입력이라 매 호출마다 from DSR_ROBOT2 import jog 하면
#       ~300~600ms 지연. 또 pressed/released 폭주 시 명령 drop 되면 멈칫.
# 해결: 단일 워커가 "현재 원하는 jog 상태" 1개만 유지.
#       set() 으로 상태만 바꾸고 즉시 리턴 → 워커가 깨어나 DSR 호출.
#       동일 상태면 재전송 안 함. 변화 시 즉시 반응.
# ════════════════════════════════════════════════════════════════════
class _JogDispatcher:
    # 이동 중 재발사 주기 — DSR 제어 주기(10ms)에 맞춰 연속 이동 보장.
    # • virtual/real 공통: jog() 는 1 제어 주기(10ms) 분량만 실행 후 정지 →
    #   10ms 마다 재발사해야 연속 이동이 가능.
    # • ramp-up 은 호출 간 속도가 누적되므로 rapid refire 로 올바르게 ramp-up 됨.
    # • 새 vel_q 로 바뀔 때만 dispatcher.set() 이 event 를 set 하므로 중복 발사
    #   는 같은 vel_q 재발사일 뿐 — DSR 은 동일 vel 연속 호출을 이상 없이 처리.
    JOG_REFIRE_S  = 0.010  # 이동 중 재발사 주기 (10ms = DSR 제어 주기)
    SAFETY_REFIRE = 1.0    # 정지 상태 재확인 주기 (DSR watchdog 대비)

    def __init__(self, jog_fn=None, log_fn=None):
        # jog_fn lazy bind — DSR_ROBOT2 import 전에도 dispatcher 생성 가능.
        # set_jog_fn() 으로 나중에 bind.
        self._jog_fn = jog_fn
        self._log = log_fn or (lambda msg: None)
        self._target = None        # (axis, ref, vel) or None=stop
        self._target_lock = threading.Lock()
        self._event = threading.Event()
        self._last_sent = None
        self._stopped = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_jog_fn(self, jog_fn):
        """DSR_ROBOT2 import 후 jog 함수 bind. 이전 set 호출들은 jog_fn None
        이라 send 못했지만, 다음 set/refire 부터 정상 발사."""
        self._jog_fn = jog_fn

    def set(self, axis, ref, vel):
        target = None if abs(float(vel)) < 1e-6 else (int(axis), int(ref), float(vel))
        with self._target_lock:
            if target == self._target:
                return
            self._target = target
        self._event.set()

    def stop(self):
        self.set(0, 0, 0)

    def shutdown(self):
        self._stopped = True
        self._event.set()

    def _send(self, target):
        if self._jog_fn is None:
            return  # DSR 미준비 — skip (다음 refire 시 시도)
        try:
            if target is None:
                self._jog_fn(0, 0, 0)
            else:
                self._jog_fn(target[0], target[1], target[2])
        except Exception as e:
            self._log(f"[jog] dispatcher 오류: {e}")

    def _loop(self):
        while not self._stopped:
            with self._target_lock:
                t_now = self._target
            # 이동 중이면 JOG_REFIRE_S(300ms) 마다 재발사 — DSR watchdog 대비
            # 정지 중이면 SAFETY_REFIRE(1.0s) 로 느슨하게 대기
            timeout = self.JOG_REFIRE_S if t_now is not None else self.SAFETY_REFIRE
            triggered = self._event.wait(timeout=timeout)
            self._event.clear()
            if self._stopped:
                break
            with self._target_lock:
                t = self._target
            if triggered and t != self._last_sent:
                # 새 target — 즉시 발사
                self._send(t)
                self._last_sent = t
            elif not triggered and t is not None:
                # 이동 중 JOG_REFIRE_S(10ms) 경과 → 재발사 (연속 이동 유지)
                self._send(t)
            elif not triggered and t is None and self._last_sent is not None:
                # 정지 SAFETY_REFIRE 경과 → 정지 재확인
                self._send(None)
                self._last_sent = None


class DsrWorker(QtCore.QObject):
    """DSR_ROBOT2 호출 전용 worker. Qt thread 에서 실행되며 시그널로 결과 통지."""

    log         = QtCore.pyqtSignal(str)
    play_started= QtCore.pyqtSignal(int)        # n_waypoints
    play_finished= QtCore.pyqtSignal(int, str)  # rc, msg
    mode_updated= QtCore.pyqtSignal(int)        # current robot_mode
    posx_received = QtCore.pyqtSignal(list)     # TCP [X, Y, Z, A, B, C]

    def __init__(self):
        super().__init__()
        self._dsr_loaded = False
        self._fns = {}
        self._busy = False        # 한 번에 하나만 실행 (블로킹 방지)
        self._service_clients = {}
        # * play 중단/일시정지 상태 — InterruptWorker 가 set, _play_impl 이 read
        self._abort_requested = False
        # * 보조 abort 이벤트 — 서비스 기반 MoveStop 이 느리거나 불가할 때도
        #   check_motion 폴링 루프가 즉시 빠져나오도록. 기존 서비스 abort 를
        #   대체하지 않고 OR 조건으로 추가 (근본: 폴링이 외부 정지에만 의존하면
        #   서비스 지연 시 사용자 abort 가 늦게 반영됨).
        self._play_abort = threading.Event()
        # * jog 전용 dispatcher — DSR import 와 무관하게 미리 생성 (lazy bind).
        #   외부 thread (DualSense) 에서 직접 set 호출 가능 → 메인 thread 시그널
        #   경유 latency 우회.
        self._jog_dispatcher = _JogDispatcher(log_fn=self.log.emit)

    def _ensure_dsr(self):
        if self._dsr_loaded:
            return
        # * DSR_ROBOT2 는 모듈 임포트 시점에 g_node = DR_init.__dsr__node 를
        #   캡처하고 _ros2_*_client 들을 module-level 에서 create_client 호출함.
        #   노드가 None 인 상태로 임포트되면 영구히 망가지므로 사전 차단.
        if _dr_get_node() is None:
            raise RuntimeError("ROS 노드가 아직 등록되지 않았습니다 (bringup 대기 중)")

        try:
            import DSR_ROBOT2
        except Exception as e:
            import traceback
            self.log.emit(f"[dsr] DSR_ROBOT2 import 실패: {type(e).__name__}: {e}")
            for line in traceback.format_exc().strip().split("\n")[-5:]:
                self.log.emit(f"[dsr]   {line}")
            raise

        # 필수 심볼 검증 후 일괄 등록 (모션은 비동기 — amove* + check_motion)
        required = [
            "amovej", "amovesj", "posj", "check_motion",
            "get_robot_mode", "set_robot_mode",
            "ROBOT_MODE_MANUAL", "ROBOT_MODE_AUTONOMOUS",
        ]
        missing = [n for n in required if not hasattr(DSR_ROBOT2, n)]
        if missing:
            raise ImportError(
                f"DSR_ROBOT2 에 다음 심볼이 없음: {missing}")

        self._fns.update({
            "amovej":          DSR_ROBOT2.amovej,
            "amovesj":         DSR_ROBOT2.amovesj,
            "posj":            DSR_ROBOT2.posj,
            "check_motion":    DSR_ROBOT2.check_motion,
            "get_robot_mode":  DSR_ROBOT2.get_robot_mode,
            "set_robot_mode":  DSR_ROBOT2.set_robot_mode,
            "MODE_MANUAL":     DSR_ROBOT2.ROBOT_MODE_MANUAL,
            "MODE_AUTONOMOUS": DSR_ROBOT2.ROBOT_MODE_AUTONOMOUS,
        })
        # 옵션 심볼 — 펌웨어/버전에 따라 없을 수 있음
        # ※ motion_pause/motion_resume 은 DSR_ROBOT2.py 에 wrapper 가 없어서
        #   InterruptWorker 가 dsr_msgs2/srv 로 직접 호출한다.
        for name in ("release_force", "release_compliance_ctrl",
                     "stop", "DR_SSTOP", "DR_QSTOP",
                     "change_operation_speed", "get_operation_speed_ratio",
                     "jog", "DR_BASE"):
            if hasattr(DSR_ROBOT2, name):
                self._fns[name] = getattr(DSR_ROBOT2, name)
        # jog 함수 확보 시 dispatcher 에 bind (외부 thread 에서 직접 호출 가능)
        if "jog" in self._fns and self._jog_dispatcher is not None:
            self._jog_dispatcher.set_jog_fn(self._fns["jog"])

        self._dsr_loaded = True
        self.log.emit(f"[dsr] _fns 등록 완료 ({len(self._fns)} 심볼)")

    def _service_ready(self, srv_path: str, timeout: float = 0.5) -> bool:
        """DSR 서비스 client 가 ready 인지 빠르게 확인. block 회피용."""
        import DSR_ROBOT2
        client_attr = {
            "get_robot_mode": "_ros2_get_robot_mode",
            "set_robot_mode": "_ros2_set_robot_mode",
        }.get(srv_path)
        if not client_attr:
            return True
        c = getattr(DSR_ROBOT2, client_attr, None)
        if c is None:
            return False
        return c.wait_for_service(timeout_sec=timeout)

    @QtCore.pyqtSlot()
    def query_mode(self):
        # 다른 작업 중이면 skip (블로킹 방지)
        if self._busy:
            return
        try:
            self._ensure_dsr()
            # 서비스 ready 가 아니면 컨트롤러가 아직 안 떴거나 죽은 것 — block 회피
            if not self._service_ready("get_robot_mode", timeout=0.3):
                msg = "[dsr] get_robot_mode 서비스 미준비 (controller 대기 중)"
                if msg != getattr(self, "_last_mode_err", None):
                    self._last_mode_err = msg
                    self.log.emit(msg)
                return
            self._busy = True
            try:
                mode = self._fns["get_robot_mode"]()
                self.mode_updated.emit(int(mode))
                # 정상 복구 시 에러 메시지 리셋
                self._last_mode_err = None
            finally:
                self._busy = False
        except RuntimeError:
            return
        except Exception as e:
            msg = f"[dsr] 모드 조회 실패: {e}"
            if msg != getattr(self, "_last_mode_err", None):
                self._last_mode_err = msg
                self.log.emit(msg)

    @QtCore.pyqtSlot()
    def go_home(self):
        if self._busy:
            self.log.emit("[dsr] ⚠️ 다른 작업 진행 중 — 홈 복귀 거부")
            return
        self._busy = True
        try:
            self._ensure_dsr()
            if not self._service_ready("set_robot_mode", timeout=1.0):
                self.log.emit("[dsr] ⚠️ controller 미준비 — 홈 복귀 거부")
                return
            self.log.emit("[dsr] 홈 복귀 시작 (async)...")
            self._fns["set_robot_mode"](self._fns["MODE_AUTONOMOUS"])
            self._fns["amovej"](
                self._fns["posj"](*HOME_POSJ), vel=30, acc=60)
            check_motion = self._fns["check_motion"]
            while check_motion():
                time.sleep(0.05)
            self.log.emit("[dsr] ✅ 홈 복귀 완료")
        except Exception as e:
            self.log.emit(f"[dsr] ⚠️ 홈 복귀 실패: {e}")
        finally:
            self._busy = False

    @QtCore.pyqtSlot(str, float, float)
    def play(self, smooth_path: str, vel: float, acc: float):
        # 다른 작업 진행 중이면 거부 (블로킹된 경우 사용자에게 알림)
        if self._busy:
            self.play_finished.emit(
                -1, "다른 DSR 작업이 진행 중입니다. 잠시 후 다시 시도하세요.")
            return
        self._busy = True
        try:
            return self._play_impl(smooth_path, vel, acc)
        finally:
            self._busy = False

    def _play_impl(self, smooth_path: str, vel: float, acc: float):
        # 새 재생 시작 — 이전 abort 이벤트 초기화 (서비스 abort 플래그와 별개)
        self._play_abort.clear()
        try:
            self._ensure_dsr()
            # 서비스 ready 확인 (controller 죽어있으면 block 안 함)
            if not self._service_ready("get_robot_mode", timeout=1.0):
                self.play_finished.emit(
                    -1, "DSR controller 서비스 미준비 — bringup 로그 확인 필요")
                return
            self.log.emit(f"[play] 로드 중: {smooth_path}")
            with open(smooth_path) as f:
                d = json.load(f)
            wps = d["waypoints_deg"]
            n = len(wps)
            self.log.emit(f"[play] {n} waypoint")
            self.log.emit(f"[play]   첫 wp: {[round(v,2) for v in wps[0]]}")
            self.log.emit(f"[play]   끝 wp: {[round(v,2) for v in wps[-1]]}")

            # ── 모드 확인 / 전환 (검증 포함)
            cur = self._fns["get_robot_mode"]()
            self.log.emit(f"[play] 현재 모드 = {MODE_NAMES.get(cur, cur)} ({cur})")
            if cur != self._fns["MODE_AUTONOMOUS"]:
                self.log.emit("[play] > AUTONOMOUS 전환 시도")
                rc_set = self._fns["set_robot_mode"](
                    self._fns["MODE_AUTONOMOUS"])
                self.log.emit(f"[play]   set_robot_mode rc={rc_set}")
                # 전환 확인 폴링 (최대 2초)
                ok = False
                for _ in range(20):
                    time.sleep(0.1)
                    if self._fns["get_robot_mode"]() == self._fns["MODE_AUTONOMOUS"]:
                        ok = True
                        break
                if not ok:
                    self.play_finished.emit(
                        -1,
                        "AUTONOMOUS 전환 실패 — 펜던트가 MANUAL 점유 중. "
                        "펜던트의 AUTO 버튼/제어권을 ROS 측으로 넘기세요.")
                    return
                self.mode_updated.emit(self._fns["MODE_AUTONOMOUS"])
                self.log.emit("[play] ✓ AUTONOMOUS 전환 완료")

            # ── amovesj 호출 (비동기) + check_motion 폴링
            self._abort_requested = False
            self.play_started.emit(n)
            pts = [self._fns["posj"](*w) for w in wps]
            self.log.emit(
                f"[play] ▶ amovesj(n={n}, vel={vel}°/s, acc={acc}°/s²) async")
            t0 = time.monotonic()
            rc = self._fns["amovesj"](pts, vel=vel, acc=acc)
            self.log.emit(f"[play] amovesj 시작 rc={rc} — check_motion 폴링")
            if rc == 0:
                check_motion = self._fns["check_motion"]
                # 종료 조건: 로봇 모션 완료(check_motion()==0) 또는
                # 보조 abort 이벤트. 서비스 기반 MoveStop 은 check_motion 을
                # 0 으로 만들어 이 루프를 빠져나오게 하므로 기존 abort 도 유효.
                while check_motion() and not self._play_abort.is_set():
                    time.sleep(0.05)
            dt = time.monotonic() - t0
            self.log.emit(f"[play] 모션 종료 (소요 {dt:.2f}s)")

            if self._abort_requested:
                msg = "중단됨 — 사용자 abort"
                rc = -2
            elif rc == 0:
                msg = "정상 완료"
            else:
                msg = f"amovesj rc={rc} — DSR 에러"
            self.play_finished.emit(int(rc), msg)
        except Exception as e:
            import traceback
            self.log.emit(f"[play] 예외: {type(e).__name__}: {e}")
            for line in traceback.format_exc().strip().split("\n")[-6:]:
                self.log.emit(f"[play]   {line}")
            self.play_finished.emit(-1, f"예외: {e}")

    @QtCore.pyqtSlot(int)
    def set_operation_speed(self, ratio: int):
        """컨트롤러 전역 속도 배율 (1-100%). amovesj/amovej의 vel/acc에 곱해짐."""
        if self._busy:
            self.log.emit("[dsr] ⚠️ busy — operation_speed 변경 거부")
            return
        self._busy = True
        try:
            self._ensure_dsr()
            if "change_operation_speed" not in self._fns:
                self.log.emit("[dsr] ⚠️ change_operation_speed 미지원 펌웨어")
                return
            if not self._service_ready("get_robot_mode", timeout=0.5):
                self.log.emit("[dsr] ⚠️ controller 미준비 — operation_speed 변경 불가")
                return
            ratio = max(1, min(100, int(ratio)))
            self._fns["change_operation_speed"](ratio)
            self.log.emit(f"[dsr] operation speed → {ratio}%")
        except Exception as e:
            self.log.emit(f"[dsr] operation_speed 변경 실패: {e}")
        finally:
            self._busy = False

    @QtCore.pyqtSlot(int)
    def set_mode(self, target_mode: int):
        """로봇 모드 전환 (0=MANUAL, 1=AUTONOMOUS).
        전환 후 폴링으로 실제 변경 검증."""
        if self._busy:
            self.log.emit("[dsr] ⚠️ busy — 모드 전환 거부")
            return
        self._busy = True
        try:
            self._ensure_dsr()
            if not self._service_ready("set_robot_mode", timeout=1.0):
                self.log.emit("[dsr] ⚠️ controller 미준비 — 모드 전환 거부")
                return
            target_name = MODE_NAMES.get(target_mode, str(target_mode))
            cur = self._fns["get_robot_mode"]()
            if cur == target_mode:
                self.log.emit(f"[dsr] 이미 {target_name} — 변경 없음")
                self.mode_updated.emit(int(cur))
                return
            self.log.emit(f"[dsr] > 모드 전환: "
                          f"{MODE_NAMES.get(cur, cur)} → {target_name}")
            rc = self._fns["set_robot_mode"](target_mode)
            self.log.emit(f"[dsr]   set_robot_mode rc={rc}")
            # 검증 — 최대 2초 폴링 (펜던트 키 스위치/제어권 점유 시 거부됨)
            ok = False
            for _ in range(20):
                time.sleep(0.1)
                if self._fns["get_robot_mode"]() == target_mode:
                    ok = True
                    break
            now = self._fns["get_robot_mode"]()
            self.mode_updated.emit(int(now))
            if ok:
                self.log.emit(f"[dsr] ✓ {target_name} 전환 완료")
            else:
                self.log.emit(
                    f"[dsr] ⚠️ {target_name} 전환 실패 — 현재 모드 "
                    f"{MODE_NAMES.get(now, now)}. "
                    "펜던트 키 스위치/제어권 확인 필요.")
        except Exception as e:
            self.log.emit(f"[dsr] 모드 전환 실패: {e}")
        finally:
            self._busy = False

    @QtCore.pyqtSlot()
    def query_posx(self):
        """TCP 좌표 1회 조회 — TCP jog 모드 표시용. busy 시 skip."""
        if self._busy:
            return
        try:
            self._ensure_dsr()
            from DSR_ROBOT2 import get_current_posx
            res = get_current_posx()
            # API: ([X,Y,Z,A,B,C], sol_space) 반환
            posx = res[0] if isinstance(res, tuple) else res
            if posx is not None and len(posx) >= 6:
                self.posx_received.emit(list(posx[:6]))
        except Exception:
            pass

    @QtCore.pyqtSlot()
    def emergency_stop(self):
        try:
            self._ensure_dsr()
            if "stop" in self._fns and "DR_SSTOP" in self._fns:
                self._fns["stop"](self._fns["DR_SSTOP"])
                self.log.emit("[dsr] 비상 정지 (DR_SSTOP)")
            else:
                self.log.emit("[dsr] ⚠️ stop 함수 미지원 펌웨어")
        except Exception as e:
            self.log.emit(f"[dsr] 비상 정지 실패: {e}")

    # ─── Jog (개별 조인트 ± 연속 제어) ───────────────────────────────
    # axis: 0~5 = J1~J6 (joint), 6~11 = X/Y/Z/A/B/C (task)
    # ref:  joint 모드는 ignored, task 모드는 0=DR_BASE / 1=DR_TOOL / 2=DR_WORLD
    # vel:  ±값. 0 이면 정지. dispatcher 가 latest-wins 로 직렬화.
    @QtCore.pyqtSlot(int, int, float)
    def jog(self, axis: int, ref: int, vel: float):
        try:
            self._ensure_dsr()
        except Exception as e:
            self.log.emit(f"[jog] DSR 미준비: {e}")
            return
        if "jog" not in self._fns:
            self.log.emit("[jog] ⚠️ DSR_ROBOT2.jog 미지원 펌웨어")
            return
        if self._busy:
            return
        self._jog_dispatcher.set(axis, ref, vel)

    @QtCore.pyqtSlot()
    def stop_jog(self):
        self._jog_dispatcher.stop()


# ════════════════════════════════════════════════════════════════════
# Interrupt Worker — pause/resume/abort 전용 (별도 스레드)
# ════════════════════════════════════════════════════════════════════
# * 왜 별도 스레드인가?
#   DsrWorker._play_impl 이 worker_thread 의 이벤트 루프를 점유한 채
#   `while check_motion(): sleep(0.05)` 로 대기 중이므로, 같은 스레드에
#   pause/resume/abort 슬롯을 얹으면 폴링이 끝난 다음에야 처리된다.
#   → 별도 스레드에서 즉시 service 호출이 가능해야 한다.
# ════════════════════════════════════════════════════════════════════
class DsrInterruptWorker(QtCore.QObject):
    log            = QtCore.pyqtSignal(str)
    paused_changed = QtCore.pyqtSignal(bool)   # True = paused, False = playing
    aborted        = QtCore.pyqtSignal()

    # * MovePause/MoveResume/MoveStop 모두 DSR_ROBOT2.py 에 함수 wrapper 가 없어서
    #   dsr_msgs2/srv 로 직접 service 호출해야 한다.
    # ※ 경로는 doosan-robot2 의 dsr_controller2.cpp 에서 실제 등록되는 이름 사용.
    #   dev-docs (`services/motion/move_stop.md`) 는 "/motion/stop" 으로 적혀 있으나
    #   소스 (cpp:2417) 는 "motion/move_stop" → 후자가 정답.
    PAUSE_SRV_PATH  = "/dsr01/motion/move_pause"
    RESUME_SRV_PATH = "/dsr01/motion/move_resume"
    STOP_SRV_PATH   = "/dsr01/motion/move_stop"
    # MoveStop.stop_mode 정수값 (DSR_ROBOT2 에서 확인됨):
    #   0=DR_QSTOP_STO  1=DR_QSTOP  2=DR_SSTOP  3=DR_HOLD
    STOP_MODE_SOFT  = 2

    def __init__(self, dsr_worker: DsrWorker):
        super().__init__()
        self._dsr = dsr_worker          # _fns 공유 참조
        self._paused = False
        self._clients = {}              # service client lazy 캐시

    def _fn(self, name: str):
        return self._dsr._fns.get(name)

    def _call_srv(self, srv_type, srv_name: str,
                  request=None, timeout: float = 2.0):
        """ROS service 직접 호출 (DSR_ROBOT2 wrapper 미제공 함수용).
        반환: (success: bool, err_msg: str|None).
        global executor (RosSpinThread) 가 callback 을 spin 하므로
        future.done() 폴링만으로 충분 (spin_until_future_complete 불필요)."""
        node = _dr_get_node()
        if node is None:
            return False, "ROS 노드 미준비"
        key = (srv_type.__name__, srv_name)
        cli = self._clients.get(key)
        if cli is None:
            cli = node.create_client(srv_type, srv_name)
            self._clients[key] = cli
        if not cli.wait_for_service(timeout_sec=1.5):
            return False, f"service unavailable: {srv_name}"
        if request is None:
            request = srv_type.Request()
        future = cli.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, f"timeout {timeout}s: {srv_name}"
        res = future.result()
        return bool(getattr(res, "success", True)), None

    @QtCore.pyqtSlot()
    def pause(self):
        if self._paused:
            return
        try:
            from dsr_msgs2.srv import MovePause
        except ImportError as e:
            self.log.emit(f"[ctrl] dsr_msgs2 import 실패: {e}")
            return
        ok, err = self._call_srv(MovePause, self.PAUSE_SRV_PATH)
        if ok:
            self._paused = True
            self.log.emit("[ctrl] 일시정지 (MovePause srv)")
            self.paused_changed.emit(True)
        else:
            self.log.emit(f"[ctrl] pause 실패: {err}")

    @QtCore.pyqtSlot()
    def resume(self):
        if not self._paused:
            return
        try:
            from dsr_msgs2.srv import MoveResume
        except ImportError as e:
            self.log.emit(f"[ctrl] dsr_msgs2 import 실패: {e}")
            return
        ok, err = self._call_srv(MoveResume, self.RESUME_SRV_PATH)
        if ok:
            self._paused = False
            self.log.emit("[ctrl] ▶ 재개 (MoveResume srv)")
            self.paused_changed.emit(False)
        else:
            self.log.emit(f"[ctrl] resume 실패: {err}")

    @QtCore.pyqtSlot()
    def abort(self):
        # MoveStop service 직접 호출 (DSR_ROBOT2.stop wrapper 가 없는 펌웨어 지원).
        # paused 상태든 playing 상태든 모두 동작.
        try:
            from dsr_msgs2.srv import MoveStop
        except ImportError as e:
            self.log.emit(f"[ctrl] dsr_msgs2 import 실패: {e}")
            return
        req = MoveStop.Request()
        req.stop_mode = self.STOP_MODE_SOFT
        # * play_finished 메시지가 "중단됨"으로 나오도록 플래그 먼저 세팅
        self._dsr._abort_requested = True
        # * 보조 abort 이벤트도 set — MoveStop 서비스가 느리거나 실패해도
        #   check_motion 폴링 루프가 즉시 빠져나옴 (기존 서비스 abort 유지).
        self._dsr._play_abort.set()
        ok, err = self._call_srv(MoveStop, self.STOP_SRV_PATH, request=req)
        if ok:
            self._paused = False
            self.log.emit("[ctrl] ✋ 중단 요청 (MoveStop srv, mode=SSTOP) — check_motion 곧 0 반환")
            self.aborted.emit()
            self.paused_changed.emit(False)
        else:
            # 실패 시 abort 플래그 롤백 (보조 이벤트도 함께 — 일관성)
            self._dsr._abort_requested = False
            self._dsr._play_abort.clear()
            self.log.emit(f"[ctrl] abort 실패: {err}")

    def is_paused(self) -> bool:
        return self._paused

    def reset(self):
        """play 종료/시작 시 내부 paused 상태 초기화."""
        self._paused = False


# ════════════════════════════════════════════════════════════════════
# ROS2 Spin 스레드
# ════════════════════════════════════════════════════════════════════
class RosSpinThread(QtCore.QThread):
    """DSR g_node 전용 spin thread. joint_states 는 JointStateThread 가 별도 처리."""
    ready          = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self._stop = False
        self.node: MacroNode | None = None
        self.executor: MultiThreadedExecutor | None = None

    def run(self):
        rclpy.init()
        self.node = MacroNode()
        _dr_set_node(self.node)             # * name-mangling 회피하여 등록

        # ★★* DSR_ROBOT2 의 spin_until_future_complete 는 rclpy.get_global_executor()
        #     를 사용한다. 우리가 별도 executor 를 만들고 거기에 노드를 등록하면,
        #     DSR 호출 시 global executor 가 add_node 를 시도하지만 노드가 이미
        #     다른 executor 에 속해 있어 False 를 반환 → global executor 는
        #     노드를 spin 하지 못하고 future 가 영원히 풀리지 않음.
        #     해결: 우리 MultiThreadedExecutor 를 rclpy 의 global executor 로 설정.
        self.executor = MultiThreadedExecutor(num_threads=4)
        rclpy.__executor = self.executor          # global executor 로 등록
        self.executor.add_node(self.node)

        self.ready.emit()
        try:
            # * executor.spin() 을 호출해야 MultiThreadedExecutor 의 thread pool 이
            #   실제로 작동한다. `spin_once + loop` 패턴은 단일 thread 처리율 (≤10Hz)
            #   로 떨어져 100Hz subscription 을 절대 못 따라잡는다 (실측 0.3Hz).
            #   spin() 은 blocking 이므로 종료는 stop() 에서 executor.shutdown() 호출.
            self.executor.spin()
        except Exception:
            pass
        finally:
            try:
                self.executor.remove_node(self.node)
                self.node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.__executor = None
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()

    def stop(self):
        self._stop = True
        try:
            if self.executor is not None:
                self.executor.shutdown()  # spin() 깨우기
        except Exception:
            pass


# ════════════════════════════════════════════════════════════════════
# JointStateThread — joint_states 구독 전용 thread + executor
# ════════════════════════════════════════════════════════════════════
class JointStateThread(QtCore.QThread):
    """별도 executor 로 joint_states 만 spin. DSR service 와 자원 경합 없음."""
    joint_received = QtCore.pyqtSignal(list)
    debug          = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.node: JointStateNode | None = None
        self.executor: SingleThreadedExecutor | None = None

    def run(self):
        # rclpy.init() 은 RosSpinThread 가 먼저 호출했음 (대기)
        for _ in range(100):
            if rclpy.ok():
                break
            time.sleep(0.05)
        if not rclpy.ok():
            return
        self.node = JointStateNode(
            on_joint=lambda j: self.joint_received.emit(j),
            on_debug=lambda s: self.debug.emit(s),
        )
        # * SingleThreadedExecutor — DSR g_node 의 MultiThreadedExecutor 와
        #   완전 독립. spin lock 경합 없이 100Hz 처리 가능.
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        try:
            self.executor.spin()
        except Exception:
            pass
        finally:
            try:
                self.executor.remove_node(self.node)
                self.node.destroy_node()
            except Exception:
                pass

    def stop(self):
        try:
            if self.executor is not None:
                self.executor.shutdown()
        except Exception:
            pass

    def diag_snapshot(self) -> dict:
        if self.node is None:
            return {"handle": 0, "success": 0, "error": 0}
        return self.node.diag_snapshot()


# ════════════════════════════════════════════════════════════════════
# 메인 윈도우
# ════════════════════════════════════════════════════════════════════
class MainWindow(QtWidgets.QMainWindow):
    request_mode         = QtCore.pyqtSignal()
    request_home         = QtCore.pyqtSignal()
    request_play         = QtCore.pyqtSignal(str, float, float)
    request_estop        = QtCore.pyqtSignal()
    request_set_ops_speed = QtCore.pyqtSignal(int)
    request_pause        = QtCore.pyqtSignal()
    request_resume       = QtCore.pyqtSignal()
    request_abort        = QtCore.pyqtSignal()
    request_set_mode     = QtCore.pyqtSignal(int)   # 0=MANUAL, 1=AUTONOMOUS
    # mini-jog: (axis 0~5=J1~J6 / 6~11=X/Y/Z/A/B/C, ref, vel) — 단계 1 은 axis 0~5 만 사용
    request_jog          = QtCore.pyqtSignal(int, int, float)
    request_stop_jog     = QtCore.pyqtSignal()
    request_posx         = QtCore.pyqtSignal()       # TCP 좌표 1회 조회 요청
    # ⚠ daemon thread → 메인 thread 의 log_view 안전 dispatch 전용
    # (외부 thread 에서 self._log() 직접 호출하면 QPlainTextEdit cross-thread
    #  access → segfault. 시그널 cross-thread queued 로 dispatch 필수.)
    _thread_log_sig      = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ros2_move_recoder — Macro Recorder")
        self.resize(1200, 760)
        self.setMinimumSize(900, 600)

        self.bringup    = BringupManager()
        self.recording  = False
        self.buffer_t: list[float]      = []
        self.buffer_q: list[list[float]] = []
        self.current_action: str | None = None
        self.last_joint: list[float]    = [0.0] * 6
        self.robot_mode: int | None     = None
        self._home_after_abort: bool    = False  # abort 종료 후 자동 홈 복귀 플래그
        self.debug_enabled: bool        = False  # 진단 로그 표시 토글 (기본 OFF)
        # mini-jog 의 "현재 선택 joint" — DualSense D-pad 가 변경, 행 강조에 반영
        self._selected_joint: int       = 0
        self._dualsense_active: bool    = False
        self._launch_mode: str          = "external"   # "real" / "virtual" / "external"
        # 그리퍼: 현재 width [mm] (None=미확인). recorder 가 매 sample 마다 buffer
        self._gripper_active: bool      = False
        self._gripper_width_mm: float | None = None
        # jog 좌표 표시 모드 — 'joint' (J1~J6 deg) / 'tcp' (X/Y/Z mm + A/B/C deg)
        self._jog_display_mode: str     = 'joint'
        self.buffer_w: list[float | None] = []   # 그리퍼 width buffer (joints 와 동일 길이)
        # play 진행 시 timeline 으로 그리퍼 명령 발사 — thread 핸들/취소 플래그
        self._grip_play_thread: threading.Thread | None = None
        self._grip_play_stop = threading.Event()

        # 테마 — 디폴트 dark. _apply_theme 가 인스턴스 생성 후 인라인 스타일까지 갱신.
        self._theme_name: str = "dark"
        self._theme: dict     = THEMES[self._theme_name]
        self._btn_variants    = btn_variants_for_theme(self._theme)

        self._build_ui()
        self._build_menu()
        self._build_statusbar()
        # 모든 위젯 생성 후 한 번 더 적용 — 인라인 스타일이 build_stylesheet 의 색을
        # override 하지 않도록 토큰 일관성 강제.
        self._apply_theme(self._theme_name)

        # Bringup 선택
        dlg = BringupDialog(self)
        dlg.exec_()
        self._launch_mode = dlg.choice  # "real" / "virtual" / "external"
        if dlg.choice in ("real", "virtual"):
            self._log(f"[bringup] {dlg.choice} 모드로 launch")
            self.bringup.launch(dlg.choice)
            self._set_status(f"Bringup({dlg.choice}) 시작 중…", "blue")
        else:
            self._log("[bringup] 외부 launch 사용")
            self._set_status("외부 bringup 사용", "gray")

        # ROS spin thread (DSR g_node 전용)
        self.ros = RosSpinThread()
        self.ros.ready.connect(self._on_ros_ready)
        self.ros.start()

        # * JointState 전용 thread — DSR executor 와 분리하여 100Hz 보장
        self.js = JointStateThread()
        self.js.joint_received.connect(self._on_joint)
        self.js.debug.connect(self._log)
        # rclpy.init() 은 RosSpinThread 가 먼저 호출하므로 약간 지연 후 시작
        QtCore.QTimer.singleShot(500, self.js.start)

        # DSR worker thread
        self.worker_thread = QtCore.QThread()
        self.dsr = DsrWorker()
        self.dsr.moveToThread(self.worker_thread)
        self.dsr.log.connect(self._log)
        self.dsr.play_started.connect(self._on_play_started)
        self.dsr.play_finished.connect(self._on_play_finished)
        self.dsr.mode_updated.connect(self._on_mode_updated)
        self.request_mode.connect(self.dsr.query_mode)
        self.request_home.connect(self.dsr.go_home)
        self.request_play.connect(self.dsr.play)
        self.request_estop.connect(self.dsr.emergency_stop)
        self.request_set_ops_speed.connect(self.dsr.set_operation_speed)
        self.request_set_mode.connect(self.dsr.set_mode)
        self.request_jog.connect(
            self.dsr.jog, QtCore.Qt.QueuedConnection)
        self.request_stop_jog.connect(
            self.dsr.stop_jog, QtCore.Qt.QueuedConnection)
        self.request_posx.connect(
            self.dsr.query_posx, QtCore.Qt.QueuedConnection)
        self.dsr.posx_received.connect(
            self._on_posx_received, QtCore.Qt.QueuedConnection)
        self.worker_thread.start()

        # TCP posx 폴링 타이머 (TCP 모드일 때만 start, 1Hz)
        self._tcp_posx_timer = QtCore.QTimer(self)
        self._tcp_posx_timer.timeout.connect(
            lambda: self.request_posx.emit())

        # ── DualSense 컨트롤러 워커 (메뉴에서 활성화 시 start) ──
        self.dualsense = DualSenseWorker(jog_max_vel=80.0)
        # ⚠ jog dispatcher 직접 참조 전달 — 시그널 경유 latency 우회
        # (DSR_ROBOT2 import 와 무관하게 dispatcher 는 미리 생성됨, jog fn 만 lazy)
        self.dualsense.set_jog_dispatcher(self.dsr._jog_dispatcher)
        # ⚠ DualSense 워커는 daemon thread — cross-thread emit. log_view 안전 위해 queued 명시.
        self.dualsense.log.connect(self._log, QtCore.Qt.QueuedConnection)
        self.dualsense.connected_changed.connect(
            self._on_dualsense_connection_changed, QtCore.Qt.QueuedConnection)
        self.dualsense.joint_selection_moved.connect(
            self._on_joint_selection_changed, QtCore.Qt.QueuedConnection)
        # 패드 입력 → mini-jog 시그널로 직접 전달 (GUI 버튼과 동일 경로)
        self.dualsense.request_jog.connect(
            self.request_jog, QtCore.Qt.QueuedConnection)
        self.dualsense.request_stop_jog.connect(
            self.request_stop_jog, QtCore.Qt.QueuedConnection)
        # ⚠ DualSense 워커는 pure Python daemon thread — Qt 의 자동 connection
        #   감지가 신뢰할 수 없음. 모든 슬롯 연결에 QueuedConnection 명시해서
        #   메인 thread 에서 슬롯이 실행되도록 보장 (modal accept/reject 안전성).
        _Q = QtCore.Qt.QueuedConnection
        self.dualsense.record_toggle.connect(
            self._on_dualsense_record_toggle, _Q)
        self.dualsense.smooth_play_combo.connect(
            self._on_dualsense_smooth_play, _Q)
        self.dualsense.pause_resume_toggle.connect(
            self._on_dualsense_pause_resume_toggle, _Q)
        self.dualsense.emergency_stop.connect(
            self._on_dualsense_estop, _Q)
        self.dualsense.home_request.connect(
            self._on_dualsense_home, _Q)
        self.dualsense.new_profile_request.connect(
            self._on_dualsense_new_profile, _Q)
        self.dualsense.mode_toggle_request.connect(
            self._on_dualsense_mode_toggle, _Q)
        self.dualsense.speed_step.connect(
            self._on_dualsense_speed_step, _Q)

        # ── OnRobot 그리퍼 워커 (메뉴에서 활성화 시 start) ──
        self.gripper = GripperWorker()
        # ⚠ Qt.QueuedConnection 명시 — gripper worker 는 daemon thread 라
        #   cross-thread emit. 위젯 access 안전하게 메인 thread 에서 처리.
        self.gripper.log.connect(self._log, _Q)
        self.gripper.connected_changed.connect(
            self._on_gripper_connection_changed, _Q)
        self.gripper.width_changed.connect(
            self._on_gripper_width_changed, _Q)
        # daemon thread (gripper play timeline 등) → 메인 thread 로그 dispatch
        self._thread_log_sig.connect(self._log, _Q)
        # □ → 그리퍼 토글 (open ↔ close)
        self.dualsense.gripper_toggle.connect(
            self._on_dualsense_gripper_toggle, _Q)
        # Options 버튼 → Joint ↔ TCP jog 모드 토글
        self.dualsense.jog_mode_changed.connect(
            self._on_dualsense_jog_mode_changed, _Q)

        # * Interrupt worker thread — play 중에도 즉시 처리되도록 별도 스레드
        self.interrupt_thread = QtCore.QThread()
        self.interrupt = DsrInterruptWorker(self.dsr)
        self.interrupt.moveToThread(self.interrupt_thread)
        self.interrupt.log.connect(self._log)
        self.interrupt.paused_changed.connect(self._on_paused_changed)
        self.interrupt.aborted.connect(self._on_aborted)
        self.request_pause.connect(self.interrupt.pause)
        self.request_resume.connect(self.interrupt.resume)
        self.request_abort.connect(self.interrupt.abort)
        self.interrupt_thread.start()

        # 모드 폴링 타이머 — ros.ready 시점부터 시작 (DSR 모듈 캐시 손상 방지)
        self.mode_timer = QtCore.QTimer(self)
        self.mode_timer.timeout.connect(lambda: self.request_mode.emit())

        # Modal 활성 모니터링 — DualSense 워커가 modal 떠있을 때 jog 차단
        self._last_modal_state: bool = False
        self._modal_timer = QtCore.QTimer(self)
        self._modal_timer.timeout.connect(self._check_modal_active)
        self._modal_timer.start(100)

        # 사이드바 새로고침
        self._refresh_action_list()

    # ─── UI 구성 ─────────────────────────────────────────
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)

        # ── 좌측 사이드바: 액션 목록 ─────
        side = QtWidgets.QGroupBox("저장된 액션 (records/)")
        side.setMaximumWidth(260)
        side_lay = QtWidgets.QVBoxLayout(side)
        self.action_list = QtWidgets.QListWidget()
        self.action_list.itemDoubleClicked.connect(self._on_load_from_list)
        side_lay.addWidget(self.action_list)
        row = QtWidgets.QHBoxLayout()
        b_refresh = QtWidgets.QPushButton("새로고침")
        b_refresh.clicked.connect(self._refresh_action_list)
        b_open = QtWidgets.QPushButton("폴더 열기")
        b_open.clicked.connect(
            lambda: subprocess.Popen(["xdg-open", str(RECORDS_DIR)]))
        b_del = QtWidgets.QPushButton("삭제")
        b_del.clicked.connect(self._delete_action)
        row.addWidget(b_refresh); row.addWidget(b_open); row.addWidget(b_del)
        side_lay.addLayout(row)
        root.addWidget(side, 0)

        # ── 우측 본체 ─────
        right = QtWidgets.QVBoxLayout()
        root.addLayout(right, 1)

        # 상단 — 액션명 + 상태
        top = QtWidgets.QHBoxLayout()
        top.setSpacing(10)
        self.action_label = QtWidgets.QLabel("액션  ·  <없음>")
        self.action_label.setStyleSheet(
            "font-size: 13pt; font-weight: 600; color: #1a1d23; letter-spacing: 0.2px;")
        self.mode_label = QtWidgets.QLabel("MODE  ?")
        self.mode_label.setStyleSheet(
            "padding:4px 10px; border-radius:10px; "
            "background:#eef0f3; color:#5b6470; "
            "font-size:9pt; font-weight:600; letter-spacing:0.6px;")
        self.mode_label.setToolTip(
            "현재 로봇 모드 — 변경은 메뉴 [모드] 또는 Ctrl+M")
        self.status_label = QtWidgets.QLabel("시작 중")
        self.status_label.setStyleSheet(
            "color: #6b7280; font-size: 10pt; font-weight: 500;")
        top.addWidget(self.action_label)
        top.addStretch()
        top.addWidget(self.mode_label)
        top.addSpacing(10)
        top.addWidget(self.status_label)
        right.addLayout(top)

        # 중단 — 좌(좌표) / 우(로그)
        mid = QtWidgets.QHBoxLayout()

        # 현재 좌표 + Mini-Jog 통합 패널
        left_box = QtWidgets.QVBoxLayout()
        cur = QtWidgets.QGroupBox(
            "현재 관절 좌표 [deg]  ·  Mini-Jog (AUTONOMOUS 전용)")
        cur_lay = QtWidgets.QGridLayout(cur)
        cur_lay.setHorizontalSpacing(6)
        cur_lay.setVerticalSpacing(4)
        # 헤더
        hdr_axis  = QtWidgets.QLabel("<b>축</b>")
        hdr_pos   = QtWidgets.QLabel("<b>현재 [deg]</b>")
        hdr_minus = QtWidgets.QLabel("<b>−</b>")
        hdr_plus  = QtWidgets.QLabel("<b>+</b>")
        hdr_vel   = QtWidgets.QLabel("<b>속도</b>")
        for w in (hdr_axis, hdr_minus, hdr_plus, hdr_vel):
            w.setAlignment(QtCore.Qt.AlignCenter)
        hdr_pos.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        for col, w in enumerate((hdr_axis, hdr_pos, hdr_minus, hdr_plus, hdr_vel)):
            cur_lay.addWidget(w, 0, col)

        self.joint_labels    = []
        self.axis_tags: list[QtWidgets.QLabel] = []   # mode 토글 시 라벨 변경
        self.jog_minus_btns: list[QtWidgets.QPushButton] = []
        self.jog_plus_btns:  list[QtWidgets.QPushButton] = []
        self.jog_vel_inputs: list[QtWidgets.QDoubleSpinBox] = []
        # 행 배치 순서: 위 → 아래 = J6 → J1 (사용자 요청, J1 이 맨 아래)
        for i in range(6):
            row = 6 - i  # i=5(J6)→row1, i=0(J1)→row6
            tag = QtWidgets.QLabel(f"<b>J{i+1}</b>")
            self.axis_tags.append(tag)
            tag.setStyleSheet(
                "color:#6b7280; font-size:10pt; font-weight:600; letter-spacing:0.5px;")
            tag.setAlignment(QtCore.Qt.AlignCenter)

            lab = QtWidgets.QLabel("—")
            lab.setStyleSheet(
                "font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
                "font-size: 14pt; color:#1a1d23; font-weight:500;"
                "letter-spacing: 0.5px;")
            lab.setMinimumWidth(110)
            lab.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            self.joint_labels.append(lab)

            btn_m = QtWidgets.QPushButton("−")
            btn_p = QtWidgets.QPushButton("+")
            for b in (btn_m, btn_p):
                b.setAutoRepeat(False)
                b.setFixedHeight(26)
                b.setFixedWidth(34)
                b.setCursor(QtCore.Qt.PointingHandCursor)
            btn_m.pressed.connect(
                lambda a=i: self._on_jog_pressed(a, -1))
            btn_m.released.connect(self._on_jog_released)
            btn_p.pressed.connect(
                lambda a=i: self._on_jog_pressed(a, +1))
            btn_p.released.connect(self._on_jog_released)

            vel_inp = QtWidgets.QDoubleSpinBox()
            vel_inp.setRange(1.0, 90.0)
            vel_inp.setSingleStep(5.0)
            vel_inp.setValue(80.0)
            vel_inp.setSuffix(" °/s")
            vel_inp.setFixedWidth(90)

            cur_lay.addWidget(tag,    row, 0)
            cur_lay.addWidget(lab,    row, 1)
            cur_lay.addWidget(btn_m,  row, 2)
            cur_lay.addWidget(btn_p,  row, 3)
            cur_lay.addWidget(vel_inp, row, 4)
            self.jog_minus_btns.append(btn_m)
            self.jog_plus_btns.append(btn_p)
            self.jog_vel_inputs.append(vel_inp)

        # 샘플 카운터 (record 진행 표시)
        sample_tag = QtWidgets.QLabel("샘플")
        sample_tag.setStyleSheet(
            "color:#6b7280; font-size:10pt; font-weight:600; letter-spacing:0.5px;")
        sample_tag.setAlignment(QtCore.Qt.AlignCenter)
        self.sample_count_label = QtWidgets.QLabel("0")
        self.sample_count_label.setStyleSheet(
            "font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
            "font-size: 12pt; color:#dc2626; font-weight:600;")
        self.sample_count_label.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        # J 행이 row 1~6 (J6 위, J1 아래) — 샘플/상태는 그 아래
        cur_lay.addWidget(sample_tag, 7, 0)
        cur_lay.addWidget(self.sample_count_label, 7, 1)
        self.jog_status_label = QtWidgets.QLabel("대기 중 — 모드 확인 후 활성")
        self.jog_status_label.setStyleSheet(
            "color:#6b7280; font-size:9pt; padding:2px;")
        cur_lay.addWidget(self.jog_status_label, 8, 0, 1, 5)

        left_box.addWidget(cur)
        # mini-jog 활성/비활성 토글 대상 — 좌표는 항상 표시, jog 위젯만 토글
        self._jog_widgets = (
            self.jog_minus_btns + self.jog_plus_btns + self.jog_vel_inputs)
        self._set_jog_enabled(False, "DSR 연결 대기 중")

        # ── 그리퍼 패널 (OnRobot RG2/RG6) ─────────────────────
        grip_box = QtWidgets.QGroupBox(
            "OnRobot 그리퍼  · DualSense 활성화 (Ctrl+D) 시 자동 연결")
        grip_lay = QtWidgets.QGridLayout(grip_box)
        grip_lay.setHorizontalSpacing(6)

        self.lbl_grip_width = QtWidgets.QLabel("— mm")
        self.lbl_grip_width.setStyleSheet(
            "font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
            "font-size: 13pt; color:#1a1d23; font-weight:600;")
        self.lbl_grip_width.setMinimumWidth(90)
        self.lbl_grip_width.setAlignment(
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.btn_grip_open  = QtWidgets.QPushButton("Open")
        self.btn_grip_close = QtWidgets.QPushButton("Close")
        self.btn_grip_move  = QtWidgets.QPushButton("Move →")
        for b in (self.btn_grip_open, self.btn_grip_close, self.btn_grip_move):
            b.setFixedHeight(28)
            b.setCursor(QtCore.Qt.PointingHandCursor)
        self.spin_grip_width = QtWidgets.QDoubleSpinBox()
        self.spin_grip_width.setRange(0.0, 160.0)   # RG6 max 160mm 까지 허용
        self.spin_grip_width.setValue(50.0)
        self.spin_grip_width.setSingleStep(5.0)
        self.spin_grip_width.setSuffix(" mm")
        self.spin_grip_width.setFixedWidth(100)

        self.btn_grip_open.clicked.connect(
            lambda: self.gripper.open() if self._gripper_active else None)
        self.btn_grip_close.clicked.connect(
            lambda: self.gripper.close() if self._gripper_active else None)
        self.btn_grip_move.clicked.connect(
            lambda: self.gripper.move(self.spin_grip_width.value())
            if self._gripper_active else None)

        grip_lay.addWidget(QtWidgets.QLabel("<b>현재</b>"), 0, 0)
        grip_lay.addWidget(self.lbl_grip_width,            0, 1)
        grip_lay.addWidget(self.btn_grip_open,             0, 2)
        grip_lay.addWidget(self.btn_grip_close,            0, 3)
        grip_lay.addWidget(QtWidgets.QLabel("<b>목표</b>"), 1, 0)
        grip_lay.addWidget(self.spin_grip_width,           1, 1)
        grip_lay.addWidget(self.btn_grip_move,             1, 2, 1, 2)

        self.lbl_grip_status = QtWidgets.QLabel(
            "비활성 — DualSense 활성화 (Ctrl+D) 시 자동 연결")
        self.lbl_grip_status.setStyleSheet(
            "color:#92400e; font-size:9pt; padding:2px;")
        grip_lay.addWidget(self.lbl_grip_status, 2, 0, 1, 4)
        left_box.addWidget(grip_box)
        self._grip_widgets = (
            self.btn_grip_open, self.btn_grip_close,
            self.btn_grip_move, self.spin_grip_width)
        self._set_gripper_ui_enabled(False, "비활성")

        # 평활화 파라미터
        params = QtWidgets.QGroupBox("평활화 파라미터")
        plw = QtWidgets.QFormLayout(params)
        self.spin_window = QtWidgets.QSpinBox()
        self.spin_window.setRange(5, 401); self.spin_window.setSingleStep(2)
        self.spin_window.setValue(51)
        self.spin_polyorder = QtWidgets.QSpinBox()
        self.spin_polyorder.setRange(1, 7); self.spin_polyorder.setValue(3)
        self.spin_maxpts = QtWidgets.QSpinBox()
        self.spin_maxpts.setRange(5, 100); self.spin_maxpts.setValue(80)
        self.spin_eps = QtWidgets.QDoubleSpinBox()
        self.spin_eps.setRange(0.1, 10.0); self.spin_eps.setSingleStep(0.1)
        self.spin_eps.setValue(0.5)
        self.spin_prom = QtWidgets.QDoubleSpinBox()
        self.spin_prom.setRange(0.1, 30.0); self.spin_prom.setSingleStep(0.5)
        self.spin_prom.setValue(2.0)
        plw.addRow("Savgol 윈도우:", self.spin_window)
        plw.addRow("다항식 차수:",   self.spin_polyorder)
        plw.addRow("최대 waypoint:", self.spin_maxpts)
        plw.addRow("정지 임계 [°]:", self.spin_eps)
        plw.addRow("전환점 prominence [°]:", self.spin_prom)
        left_box.addWidget(params)

        # ── 재생 속도 ────────────────────────────────────
        speed_box = QtWidgets.QGroupBox("재생 속도")
        sp_lay = QtWidgets.QVBoxLayout(speed_box)

        # vel/acc 직접 입력
        sp_form = QtWidgets.QFormLayout()
        self.spin_vel = QtWidgets.QDoubleSpinBox()
        self.spin_vel.setRange(1, 360); self.spin_vel.setSingleStep(5)
        self.spin_vel.setValue(60)
        self.spin_vel.setSuffix(" °/s")
        self.spin_acc = QtWidgets.QDoubleSpinBox()
        self.spin_acc.setRange(1, 720); self.spin_acc.setSingleStep(10)
        self.spin_acc.setValue(120)
        self.spin_acc.setSuffix(" °/s²")
        sp_form.addRow("vel:", self.spin_vel)
        sp_form.addRow("acc:", self.spin_acc)
        sp_lay.addLayout(sp_form)

        # 프리셋 버튼 — 균일 톤, 누름 시 액센트.
        # 스타일은 _restyle_static_widgets 가 테마별로 적용 (여기선 객체만 만든다).
        preset_row = QtWidgets.QHBoxLayout()
        preset_row.setSpacing(6)
        self._preset_buttons = []
        for label, vel, acc in [
            ("Slow",    15,  30),
            ("Normal",  60, 120),
            ("Fast",   120, 240),
            ("Max",    240, 480),
        ]:
            b = QtWidgets.QPushButton(label)
            b.setCursor(QtCore.Qt.PointingHandCursor)
            b.clicked.connect(
                lambda _, v=vel, a=acc, n=label:
                    self._apply_speed_preset(v, a, n))
            preset_row.addWidget(b)
            self._preset_buttons.append(b)
        sp_lay.addLayout(preset_row)

        # 시연 속도 기반 추천 — secondary outlined, 인디고 텍스트.
        # 스타일은 _restyle_static_widgets 가 테마별로 적용.
        b_auto = QtWidgets.QPushButton("시연 속도 기반 자동 추천")
        b_auto.setCursor(QtCore.Qt.PointingHandCursor)
        self.btn_auto_speed = b_auto
        b_auto.clicked.connect(self._suggest_speed_from_raw)
        sp_lay.addWidget(b_auto)

        # Operation Speed 슬라이더 (컨트롤러 전역 배율)
        ops_row = QtWidgets.QHBoxLayout()
        ops_row.addWidget(QtWidgets.QLabel("Operation Speed:"))
        self.slider_ops = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_ops.setRange(1, 100)
        self.slider_ops.setValue(100)
        self.slider_ops.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.slider_ops.setTickInterval(10)
        self.lbl_ops = QtWidgets.QLabel("100%")
        self.lbl_ops.setMinimumWidth(48)
        self.lbl_ops.setStyleSheet(
            "font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
            "font-weight:600; color:#4f46e5; font-size:10pt;")
        # 드래그 중 라벨만 갱신, 손 떼면 컨트롤러 호출 (호출 폭주 방지)
        self.slider_ops.valueChanged.connect(self._on_ops_slider_changed)
        self.slider_ops.sliderReleased.connect(self._on_ops_slider_released)
        ops_row.addWidget(self.slider_ops, 1)
        ops_row.addWidget(self.lbl_ops)
        sp_lay.addLayout(ops_row)

        # 실효 속도 표시
        self.lbl_effective = QtWidgets.QLabel("")
        self.lbl_effective.setStyleSheet(
            "color:#8b95a3; "
            "font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
            "font-size:9pt; padding:2px 4px;")
        self.spin_vel.valueChanged.connect(self._update_effective_label)
        sp_lay.addWidget(self.lbl_effective)
        self._update_effective_label()

        left_box.addWidget(speed_box)
        left_box.addStretch()
        # 좌측 패널을 QScrollArea 로 감싸기 — 그룹박스가 많아 화면이 작으면
        # 하단(평활화 파라미터/재생 속도) 가 잘리던 문제 해결.
        left_widget = QtWidgets.QWidget()
        left_widget.setLayout(left_box)
        left_scroll = QtWidgets.QScrollArea()
        left_scroll.setWidget(left_widget)
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        left_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        left_scroll.setMinimumWidth(380)
        mid.addWidget(left_scroll, 0)

        # 우측 — 로그
        log_box = QtWidgets.QGroupBox("기록 좌표 / 시스템 로그")
        log_lay = QtWidgets.QVBoxLayout(log_box)
        # ── 상단 헤더: 디버그 토글 (작게, 우측 정렬) ──
        log_header = QtWidgets.QHBoxLayout()
        log_header.setContentsMargins(0, 0, 0, 0)
        log_header.addStretch()
        self.btn_debug_toggle = QtWidgets.QPushButton("DEBUG: OFF")
        self.btn_debug_toggle.setCheckable(True)
        self.btn_debug_toggle.setChecked(False)
        self.btn_debug_toggle.setToolTip(
            "기록 중 상세 진단 로그 (subscription Hz, sample 단위 좌표 등) 표시 토글")
        self._update_debug_btn_style()
        self.btn_debug_toggle.toggled.connect(self._on_debug_toggle)
        log_header.addWidget(self.btn_debug_toggle)
        log_lay.addLayout(log_header)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setStyleSheet(
            "QPlainTextEdit {"
            "  font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
            "  font-size: 9pt; line-height: 1.4;"
            "  background: #15171c; color: #d4d7dc;"
            "  border: 1px solid #24272d; border-radius: 6px;"
            "  padding: 8px; selection-background-color: #4f46e5; }"
        )
        self.log_view.setMaximumBlockCount(3000)
        log_lay.addWidget(self.log_view)
        mid.addWidget(log_box, 1)
        right.addLayout(mid, 1)

        # 진행률 바 (재생 시)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setTextVisible(True)
        self.progress.setFormat("%v / %m waypoint")
        self.progress.setValue(0)
        right.addWidget(self.progress)

        # 하단 — 버튼들 (1행: 워크플로 1→2→3→4 + PAUSE/RESUME, 2행: ABORT/HOME/E-STOP)
        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(8)
        # 메인 워크플로 — 번호로 순서 명시
        self.btn_record = self._mkbtn("01   RECORD", "primary", self._on_record)
        self.btn_stop   = self._mkbtn("02   STOP",   "neutral", self._on_stop)
        self.btn_smooth = self._mkbtn("03   SMOOTH", "primary", self._on_smooth)
        self.btn_play   = self._mkbtn("04   PLAY",   "primary", self._on_play)
        self.btn_pause  = self._mkbtn("PAUSE",  "neutral", self._on_pause)
        self.btn_resume = self._mkbtn("RESUME", "neutral", self._on_resume)

        self.btn_stop.setEnabled(False)
        self.btn_smooth.setEnabled(False)
        self.btn_play.setEnabled(False)
        self.btn_pause.setEnabled(False)
        self.btn_resume.setEnabled(False)
        for b in (self.btn_record, self.btn_stop, self.btn_smooth,
                  self.btn_play, self.btn_pause, self.btn_resume):
            btn_row.addWidget(b)
        right.addLayout(btn_row)

        # ── 2행 = 안전/유틸 (ABORT, HOME, E-STOP)
        ctrl_row = QtWidgets.QHBoxLayout()
        ctrl_row.setSpacing(8)
        self.btn_abort = self._mkbtn("ABORT",  "neutral", self._on_abort)
        self.btn_home  = self._mkbtn("HOME",   "neutral", self._on_home)
        self.btn_estop = self._mkbtn("E-STOP", "danger",  self._on_estop)
        self.btn_abort.setEnabled(False)
        for b in (self.btn_abort, self.btn_home, self.btn_estop):
            ctrl_row.addWidget(b)
        right.addLayout(ctrl_row)

    # ── 버튼 variant 시스템 ──────────────────────────────
    #   primary   : accent 채움 — 워크플로 핵심 (RECORD/SMOOTH/PLAY)
    #   neutral   : surface + 보더 — 보조/제어 (STOP/PAUSE/RESUME/HOME)
    #   danger    : 적색 채움 — 위험/긴급 (ABORT/E-STOP)
    # self._btn_variants 는 _apply_theme 에서 동적으로 채워짐 (테마별).

    def _apply_btn_style(self, b, variant):
        """버튼에 variant 기반 stylesheet 적용 — 테마 변경 시 재호출."""
        v = self._btn_variants.get(variant, self._btn_variants["neutral"])
        t = self._theme
        b.setStyleSheet(
            f"QPushButton {{"
            f"  background:{v['bg']}; color:{v['fg']};"
            f"  border:1px solid {v['border']};"
            f"  padding:12px 14px; font-size:10.5pt; font-weight:600;"
            f"  border-radius:6px; letter-spacing:0.3px;"
            f"}}"
            f"QPushButton:hover:!disabled {{"
            f"  background:{v['hover_bg']}; border-color:{v['hover_border']};"
            f"}}"
            f"QPushButton:pressed:!disabled {{"
            f"  background:{v['press_bg']};"
            f"}}"
            f"QPushButton:disabled {{"
            f"  background:{t['btn_disabled_bg']};"
            f"  color:{t['btn_disabled_text']};"
            f"  border-color:{t['btn_disabled_border']};"
            f"}}"
        )

    def _mkbtn(self, label, variant, slot):
        """워크플로/제어용 큰 버튼 생성. variant: 'primary'|'neutral'|'danger'.
        버튼에 _variant 속성을 박아서 테마 변경 시 _apply_btn_style 로 재스타일."""
        b = QtWidgets.QPushButton(label)
        b.setCursor(QtCore.Qt.PointingHandCursor)
        b._variant = variant   # 테마 변경 시 자동 재스타일 위한 태그
        self._apply_btn_style(b, variant)
        b.clicked.connect(slot)
        return b

    def _build_menu(self):
        m = self.menuBar()
        f = m.addMenu("&파일")

        a_open = QtWidgets.QAction("폴더 열기 (액션 로드)…", self)
        a_open.setShortcut("Ctrl+O")
        a_open.triggered.connect(self._open_action_folder)
        f.addAction(a_open)

        a_browse = QtWidgets.QAction("records 폴더 보기", self)
        a_browse.triggered.connect(
            lambda: subprocess.Popen(["xdg-open", str(RECORDS_DIR)]))
        f.addAction(a_browse)

        f.addSeparator()
        a_exit = QtWidgets.QAction("종료", self)
        a_exit.setShortcut("Ctrl+Q")
        a_exit.triggered.connect(self.close)
        f.addAction(a_exit)

        # ── 모드 메뉴 ─────────────────────────────────────
        mm = m.addMenu("&모드")

        # 단일 토글 액션 — 라벨/enabled 는 _apply_mode_toggle_style 에서 동적 갱신
        self.act_mode_toggle = QtWidgets.QAction("모드 전환", self)
        self.act_mode_toggle.setShortcut("Ctrl+M")
        self.act_mode_toggle.triggered.connect(self._on_toggle_mode)
        mm.addAction(self.act_mode_toggle)

        mm.addSeparator()

        # 명시적 액션 — 현재 모드는 disabled, 다른 모드만 활성
        self.act_mode_manual = QtWidgets.QAction("MANUAL 모드로 전환", self)
        self.act_mode_manual.setCheckable(True)
        self.act_mode_manual.triggered.connect(
            lambda: self._request_mode_change(MODE_MANUAL))
        mm.addAction(self.act_mode_manual)

        self.act_mode_auto = QtWidgets.QAction("AUTONOMOUS 모드로 전환", self)
        self.act_mode_auto.setCheckable(True)
        self.act_mode_auto.triggered.connect(
            lambda: self._request_mode_change(MODE_AUTONOMOUS))
        mm.addAction(self.act_mode_auto)

        # 메뉴 액션 초기 상태 적용
        self._apply_mode_toggle_style(self.robot_mode)

        # ── 컨트롤러 메뉴 — DualSense 활성화 토글 ─────────
        cm = m.addMenu("&컨트롤러")
        self.act_dualsense = QtWidgets.QAction("DualSense 활성화", self)
        self.act_dualsense.setCheckable(True)
        self.act_dualsense.setChecked(False)
        self.act_dualsense.setShortcut("Ctrl+D")
        self.act_dualsense.toggled.connect(self._on_toggle_dualsense)
        cm.addAction(self.act_dualsense)

        cm.addSeparator()

        # 디버그 모드 — verbose 입력 로그 (button up, hat/axis 변화, 헬스 통계)
        self.act_dualsense_debug = QtWidgets.QAction(
            "DualSense 디버그 모드", self)
        self.act_dualsense_debug.setCheckable(True)
        self.act_dualsense_debug.setChecked(False)
        self.act_dualsense_debug.setShortcut("Ctrl+Shift+D")
        self.act_dualsense_debug.setToolTip(
            "DualSense 입력 자세히 로그 출력 (button up, hat/axis 변화, "
            "L3/R3 상태, L2/R2 raw, 폴링 헬스 통계, 폴링 지연 워닝)")
        self.act_dualsense_debug.toggled.connect(
            self._on_toggle_dualsense_debug)
        cm.addAction(self.act_dualsense_debug)

        # 매핑 cheat sheet 표시
        cm.addSeparator()
        a_cheatsheet = QtWidgets.QAction("DualSense 매핑 보기…", self)
        a_cheatsheet.triggered.connect(self._on_show_dualsense_cheatsheet)
        cm.addAction(a_cheatsheet)

        # ── 보기 메뉴 — 테마 (Light / Dark) ───────────────
        vm = m.addMenu("&보기")
        theme_group = QtWidgets.QActionGroup(self)
        theme_group.setExclusive(True)

        self.act_theme_dark = QtWidgets.QAction("다크 모드", self)
        self.act_theme_dark.setCheckable(True)
        self.act_theme_dark.setChecked(self._theme_name == "dark")
        self.act_theme_dark.setShortcut("Ctrl+Shift+D")
        self.act_theme_dark.triggered.connect(lambda: self._on_theme_change("dark"))
        theme_group.addAction(self.act_theme_dark)
        vm.addAction(self.act_theme_dark)

        self.act_theme_light = QtWidgets.QAction("라이트 모드", self)
        self.act_theme_light.setCheckable(True)
        self.act_theme_light.setChecked(self._theme_name == "light")
        self.act_theme_light.setShortcut("Ctrl+Shift+L")
        self.act_theme_light.triggered.connect(lambda: self._on_theme_change("light"))
        theme_group.addAction(self.act_theme_light)
        vm.addAction(self.act_theme_light)

        # 단축키
        self.btn_record.setShortcut("R")
        self.btn_stop.setShortcut("S")
        self.btn_smooth.setShortcut("M")
        self.btn_play.setShortcut("P")
        self.btn_pause.setShortcut("Space")
        self.btn_resume.setShortcut("Shift+Space")
        self.btn_abort.setShortcut("X")
        self.btn_estop.setShortcut("Esc")

    def _build_statusbar(self):
        sb = self.statusBar()
        self.sb_node = QtWidgets.QLabel("Node: 시작 중…")
        self.sb_topic = QtWidgets.QLabel("/dsr01/joint_states: ?")
        self.sb_action = QtWidgets.QLabel("")
        # 🎮 DualSense 인디케이터 — 비활성/연결안됨/연결됨 3 상태
        self.sb_dualsense = QtWidgets.QLabel()
        self._set_dualsense_pill(active=False, connected=False, name="")
        # 🦾 Gripper 인디케이터
        self.sb_gripper = QtWidgets.QLabel()
        self._set_gripper_pill(active=False, connected=False, width_mm=None)
        sb.addPermanentWidget(self.sb_node)
        sb.addPermanentWidget(QtWidgets.QLabel(" │ "))
        sb.addPermanentWidget(self.sb_topic)
        sb.addPermanentWidget(QtWidgets.QLabel(" │ "))
        sb.addPermanentWidget(self.sb_dualsense)
        sb.addPermanentWidget(QtWidgets.QLabel(" │ "))
        sb.addPermanentWidget(self.sb_gripper)
        sb.addPermanentWidget(QtWidgets.QLabel(" │ "))
        sb.addPermanentWidget(self.sb_action)

        self._joint_count = 0
        self._joint_t_last = time.time()
        self._joint_hz = 0.0
        self._hz_timer = QtCore.QTimer(self)
        self._hz_timer.timeout.connect(self._update_hz)
        self._hz_timer.start(1000)

    # ─── 사이드바 / 액션 목록 ────────────────────────────
    def _refresh_action_list(self):
        self.action_list.clear()
        if not RECORDS_DIR.exists():
            return
        for d in sorted(RECORDS_DIR.iterdir()):
            if not d.is_dir():
                continue
            tag = []
            if (d / "raw.json").exists():    tag.append("raw")
            if (d / "smooth.json").exists(): tag.append("smooth")
            label = f"{d.name}  [{'+'.join(tag)}]" if tag else d.name
            item = QtWidgets.QListWidgetItem(label)
            item.setData(QtCore.Qt.UserRole, d.name)
            self.action_list.addItem(item)

    def _on_load_from_list(self, item):
        name = item.data(QtCore.Qt.UserRole)
        self._load_action(name)

    def _open_action_folder(self):
        path = QtWidgets.QFileDialog.getExistingDirectory(
            self, "액션 폴더 선택", str(RECORDS_DIR))
        if not path:
            return
        self._load_action(Path(path).name)

    def _load_action(self, name: str):
        d = RECORDS_DIR / name
        if not d.is_dir():
            QtWidgets.QMessageBox.warning(self, "로드", f"폴더 없음: {d}")
            return
        self.current_action = name
        self.action_label.setText(f"액션: <b>{name}</b>")
        self.btn_smooth.setEnabled((d / "raw.json").exists())
        self.btn_play.setEnabled((d / "smooth.json").exists())
        self._log(f"[load] '{name}' 로드")
        # 메타 표시
        info = []
        if (d / "raw.json").exists():
            r = json.loads((d / "raw.json").read_text())
            info.append(f"raw: {r['samples']}샘플 / {r['duration_sec']}s")
        if (d / "smooth.json").exists():
            s = json.loads((d / "smooth.json").read_text())
            info.append(f"smooth: {s['n_waypoints']}wp")
            # smooth.json 의 vel/acc 를 GUI 에 동기화 (액션마다 다른 속도 보존)
            try:
                v = float(s.get("vel", self.spin_vel.value()))
                a = float(s.get("acc", self.spin_acc.value()))
                self.spin_vel.setValue(v)
                self.spin_acc.setValue(a)
                self._log(f"[load] smooth.json 속도 적용: vel={v}°/s, acc={a}°/s²")
            except (TypeError, ValueError):
                pass
        self.sb_action.setText(" / ".join(info))
        self._set_status(f"로드: {name}", "blue")

    def _delete_action(self):
        item = self.action_list.currentItem()
        if not item:
            return
        name = item.data(QtCore.Qt.UserRole)
        ans = QtWidgets.QMessageBox.question(
            self, "삭제 확인",
            f"'{name}' 폴더를 삭제하시겠습니까?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        if ans != QtWidgets.QMessageBox.Yes:
            return
        import shutil
        shutil.rmtree(RECORDS_DIR / name, ignore_errors=True)
        self._log(f"[delete] {name}")
        self._refresh_action_list()

    # ─── 시그널 핸들러 ───────────────────────────────────
    def _on_ros_ready(self):
        self.sb_node.setText("Node: ✓ macro_gui_node")
        self._set_status("대기 중", "gray")
        # ROS 노드 등록 완료 → 이제부터 DSR_ROBOT2 안전하게 임포트 가능
        # 첫 모드 조회를 약간 지연시켜 dsr_controller2 의 service ready 대기
        QtCore.QTimer.singleShot(2000, lambda: self.request_mode.emit())
        self.mode_timer.start(3000)
        self._log("[ros] 노드 ready — DSR 서비스 폴링 시작")

    def _on_joint(self, joints: list):
        self.last_joint = joints
        # joint 좌표 표시는 'joint' 모드일 때만 (TCP 모드는 posx 별도 polling)
        if self._jog_display_mode == 'joint':
            for i, lab in enumerate(self.joint_labels):
                lab.setText(f"{joints[i]:>+8.3f}")
        self._joint_count += 1
        if self.recording:
            t = time.monotonic()
            self.buffer_t.append(t)
            self.buffer_q.append(joints)
            # 그리퍼 활성 + width 알려진 경우만 기록. 미연결/미활성이면 None.
            self.buffer_w.append(
                self._gripper_width_mm if self._gripper_active else None)
            n = len(self.buffer_q)
            self.sample_count_label.setText(str(n))
            # * 디버그: 첫 10개 + 50 마다 (DEBUG ON 일 때만 출력)
            if n <= 10 or n % 50 == 0:
                vals = "  ".join(f"{v:>+7.2f}" for v in joints)
                self._log_debug(f"  [rec #{n:>4}] {vals}")

    def _update_hz(self):
        now = time.time()
        dt = now - self._joint_t_last
        if dt > 0:
            self._joint_hz = self._joint_count / dt
        self._joint_count = 0
        self._joint_t_last = now
        self.sb_topic.setText(f"/dsr01/joint_states: {self._joint_hz:.1f} Hz")

    def _on_mode_updated(self, mode: int):
        self.robot_mode = mode
        name = MODE_NAMES.get(mode, f"?{mode}")
        # 톤다운된 pill — 옅은 배경 + 짙은 텍스트
        bg, fg = {
            "MANUAL":     ("#fef3c7", "#92400e"),  # amber
            "AUTONOMOUS": ("#d1fae5", "#065f46"),  # emerald
        }.get(name, ("#eef0f3", "#5b6470"))
        # 텍스트만 갱신 — 색상은 _on_mode_updated_style_only(테마 토큰 기반) 가 담당.
        self.mode_label.setText(f"MODE  {name}")
        self._on_mode_updated_style_only(mode)
        self._apply_mode_toggle_style(mode)
        # mini-jog 는 AUTONOMOUS 에서만 가능
        self._refresh_jog_enabled()

    def _apply_mode_toggle_style(self, current_mode):
        """현재 모드에 따라 메뉴의 모드 액션 라벨/체크/활성 상태 갱신.

        - act_mode_toggle : '현재 → 대상' 한 줄로 표시, Ctrl+M 단축키
        - act_mode_manual / act_mode_auto : 현재 모드는 체크 + 비활성, 반대편만 활성
        """
        # _build_ui 에서 이 함수를 _build_menu 보다 먼저 호출할 수 있으므로 가드.
        if not hasattr(self, "act_mode_toggle"):
            return
        if current_mode == MODE_MANUAL:
            self.act_mode_toggle.setText("MANUAL  →  AUTONOMOUS 로 전환")
            self.act_mode_toggle.setEnabled(True)
        elif current_mode == MODE_AUTONOMOUS:
            self.act_mode_toggle.setText("AUTONOMOUS  →  MANUAL 로 전환")
            self.act_mode_toggle.setEnabled(True)
        else:
            self.act_mode_toggle.setText("모드 전환 (현재 모드 확인 중)")
            self.act_mode_toggle.setEnabled(False)

        is_manual = (current_mode == MODE_MANUAL)
        is_auto   = (current_mode == MODE_AUTONOMOUS)
        self.act_mode_manual.setChecked(is_manual)
        self.act_mode_manual.setEnabled(current_mode is not None and not is_manual)
        self.act_mode_auto.setChecked(is_auto)
        self.act_mode_auto.setEnabled(current_mode is not None and not is_auto)

    def _request_mode_change(self, target: int):
        """명시적 모드 액션(MANUAL 로 전환 / AUTO 로 전환) 진입점."""
        if self.dsr._busy or self.interrupt.is_paused():
            QtWidgets.QMessageBox.warning(
                self, "모드 전환",
                "재생/일시정지 중에는 모드 전환할 수 없습니다.\n"
                "먼저 ABORT 또는 재생 완료 후 시도하세요.")
            # 체크 상태 원복
            self._apply_mode_toggle_style(self.robot_mode)
            return
        if self.robot_mode == target:
            return
        target_name = MODE_NAMES.get(target, str(target))
        self._log(f"[ctrl] 모드 전환 요청 → {target_name}")
        self.request_set_mode.emit(target)

    def _on_toggle_mode(self):
        # 재생/일시정지 중에는 거부
        if self.dsr._busy or self.interrupt.is_paused():
            QtWidgets.QMessageBox.warning(
                self, "모드 전환",
                "재생/일시정지 중에는 모드 전환할 수 없습니다.\n"
                "먼저 ABORT 또는 재생 완료 후 시도하세요.")
            return
        # 현재 모드의 반대로 전환
        if self.robot_mode == MODE_MANUAL:
            target = MODE_AUTONOMOUS
            target_name = "AUTONOMOUS"
        elif self.robot_mode == MODE_AUTONOMOUS:
            target = MODE_MANUAL
            target_name = "MANUAL"
        else:
            self._log("[mode] 현재 모드를 모름 — 전환 거부")
            return
        self._log(f"[mode] {target_name} 전환 요청 (토글)")
        self.request_set_mode.emit(target)

    # ─── 버튼: Record / Stop ─────────────────────────────
    def _on_record(self):
        if self.recording:
            return
        if self.robot_mode is not None and self.robot_mode != MODE_MANUAL:
            ans = QtWidgets.QMessageBox.question(
                self, "Record",
                f"현재 모드: {MODE_NAMES.get(self.robot_mode, self.robot_mode)}\n"
                "기록은 보통 MANUAL 모드(펜던트 직접 조작)에서 합니다.\n"
                "그래도 진행할까요?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
            if ans != QtWidgets.QMessageBox.Yes:
                return

        name, ok = QtWidgets.QInputDialog.getText(
            self, "Record", "액션 이름을 입력하세요:",
            text=f"action_{datetime.now():%Y%m%d_%H%M%S}")
        if not ok or not name.strip():
            return
        name = name.strip().replace(" ", "_")
        self._start_record(name)

    def _start_record(self, name: str):
        """이름이 결정된 상태에서 record 즉시 시작.
        _on_record (GUI 버튼) 와 DualSense 새 프로파일 long-press 가 공유."""
        self.current_action = name
        self.action_label.setText(f"액션: <b>{name}</b>")

        self.buffer_t.clear()
        self.buffer_q.clear()
        self.buffer_w.clear()
        self.sample_count_label.setText("0")
        self.log_view.clear()
        self._log(f"[record] '{name}' 기록 시작")
        if self._gripper_active and self.gripper.is_connected():
            self._log(
                f"[record] 그리퍼 width 함께 기록 "
                f"(현재 {self._gripper_width_mm}mm)")

        # * 진단 baseline — 기록 시작 시점의 노드 콜백 통계 스냅샷
        self._rec_t0 = time.monotonic()
        self._rec_node_handle0 = 0
        self._rec_node_success0 = 0
        self._rec_node_error0 = 0
        if self.js.node is not None:
            snap = self.js.node.diag_snapshot()
            self._rec_node_handle0 = snap["handle"]
            self._rec_node_success0 = snap["success"]
            self._rec_node_error0 = snap["error"]
            self._log_debug(
                f"[record][diag] 시작 baseline — "
                f"노드 누적 handle={snap['handle']}, success={snap['success']}, "
                f"error={snap['error']}, recording={self.recording}, "
                f"buffer_q={len(self.buffer_q)}")

        self.recording = True
        self.btn_record.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_smooth.setEnabled(False)
        self.btn_play.setEnabled(False)
        self._set_status("REC", "red")

        # * 1Hz 진단 타이머 — 기록 중 매 초 통계 로그
        self._rec_diag_timer = QtCore.QTimer(self)
        self._rec_diag_timer.timeout.connect(self._record_diag_tick)
        self._rec_diag_timer.start(1000)

    def _on_stop(self):
        if not self.recording:
            return
        self.recording = False
        self.btn_record.setEnabled(True)
        self.btn_stop.setEnabled(False)

        # * 진단 타이머 종료 + 최종 비교 통계 출력
        try:
            self._rec_diag_timer.stop()
        except Exception:
            pass
        if self.js.node is not None:
            snap = self.js.node.diag_snapshot()
            elapsed = time.monotonic() - self._rec_t0
            d_handle = snap["handle"] - self._rec_node_handle0
            d_success = snap["success"] - self._rec_node_success0
            d_error = snap["error"] - self._rec_node_error0
            n_buffer = len(self.buffer_q)
            self._log_debug(
                f"[record][diag] 종료 요약 ({elapsed:.1f}s) — "
                f"노드 handle Δ={d_handle} ({d_handle/elapsed:.1f}Hz), "
                f"success Δ={d_success}, error Δ={d_error}, "
                f"GUI buffer={n_buffer}")
            if d_success > 0 and n_buffer < d_success * 0.5:
                self._log_debug(
                    f"[record][diag] GUI buffer 누락 의심 — "
                    f"노드는 {d_success}개 처리했는데 GUI 는 {n_buffer}개만 적재. "
                    f"signal queue overflow / GUI thread 지연 가능성.")
            if d_error > 0:
                self._log_debug(
                    f"[record][diag] 좌표 추출 실패 {d_error}건 — "
                    f"msg.name 형식 문제로 의심됨.")

        if not self.buffer_q:
            self._log("[record] 데이터 없음 — 저장 안 함")
            self._set_status("대기", "gray")
            return
        path = self._save_raw()
        self._log(f"[record] 저장: {path}")
        self._set_status(
            f"기록 완료 ({len(self.buffer_q)} 샘플)", "green")
        self.btn_smooth.setEnabled(True)
        self._refresh_action_list()

    def _record_diag_tick(self):
        """기록 중 1Hz 진단 — 노드 콜백 통계 vs GUI buffer 비교."""
        if not self.recording or self.js.node is None:
            return
        snap = self.js.node.diag_snapshot()
        elapsed = time.monotonic() - self._rec_t0
        d_handle = snap["handle"] - self._rec_node_handle0
        d_success = snap["success"] - self._rec_node_success0
        d_error = snap["error"] - self._rec_node_error0
        n_buffer = len(self.buffer_q)
        self._log_debug(
            f"[rec][diag][{elapsed:>5.1f}s] "
            f"node handle={d_handle} succ={d_success} err={d_error} "
            f"({d_success/elapsed:.1f}Hz unique) │ "
            f"GUI buf={n_buffer} ({n_buffer/elapsed:.1f}Hz)")

    def _save_raw(self) -> Path:
        action_dir = RECORDS_DIR / self.current_action
        action_dir.mkdir(parents=True, exist_ok=True)
        out = action_dir / "raw.json"
        t0 = self.buffer_t[0]
        timestamps_ms = [int((t - t0) * 1000) for t in self.buffer_t]
        duration = self.buffer_t[-1] - t0
        rate = len(self.buffer_q) / duration if duration > 0 else 0.0
        data = {
            "action_name": self.current_action,
            "timestamps_ms": timestamps_ms,
            "joints_deg": self.buffer_q,
            "rate_hz_avg": round(rate, 2),
            "duration_sec": round(duration, 3),
            "samples": len(self.buffer_q),
            "recorded_at": datetime.now(timezone.utc).isoformat(),
        }
        # 그리퍼 width 가 1개라도 기록되었으면 함께 저장 (None 은 그대로 보존)
        if self.buffer_w and any(w is not None for w in self.buffer_w):
            data["gripper_widths_mm"] = [
                (round(w, 2) if w is not None else None) for w in self.buffer_w]
        out.write_text(json.dumps(data, indent=2))
        return out

    # ─── 버튼: Smooth ────────────────────────────────────
    def _on_smooth(self):
        if not self.current_action:
            return
        self._log(f"[smooth] '{self.current_action}' 평활화 시작...")
        try:
            r = smooth_and_save(
                self.current_action,
                window=self.spin_window.value(),
                polyorder=self.spin_polyorder.value(),
                max_pts=self.spin_maxpts.value(),
                vel=self.spin_vel.value(),
                acc=self.spin_acc.value(),
                stationary_eps_deg=self.spin_eps.value(),
                turning_prominence_deg=self.spin_prom.value(),
            )

            # 단계별 로그
            self._log(
                f"[smooth] > Savitzky-Golay 평활화: "
                f"window={r['window']} polyorder={r['polyorder']}")
            self._log(
                f"[smooth] > 정지 구간 제거 (eps={r['stationary_eps']}°): "
                f"{r['n_raw']} → {r['n_after_stationary']} 샘플 "
                f"({r['n_raw'] - r['n_after_stationary']} 개 제거)")
            self._log(
                f"[smooth] > 방향 전환점 검출 (prominence≥{r['turning_prominence']}°): "
                f"{r['n_turning']} 개 강제 보존")
            self._log(
                f"[smooth] > arc-length + 전환점 union: "
                f"{r['n_after_stationary']} → {r['n_waypoints']} waypoint")
            self._log(
                f"[smooth] > 인접 점프: 평균 {r['avg_jump']}°  "
                f"max {r['max_jump']}°")

            # 관절별 범위
            jr_str = "  ".join(
                f"J{i+1}[{lo:>+6.1f}~{hi:>+6.1f}]"
                for i, (lo, hi) in enumerate(r["joint_ranges"]))
            self._log(f"[smooth] > 관절 범위: {jr_str}")

            # 시작/끝 자세
            f0 = "  ".join(f"{v:>+7.2f}" for v in r["wp_first"])
            fN = "  ".join(f"{v:>+7.2f}" for v in r["wp_last"])
            self._log(f"[smooth] > 시작: [{f0}]")
            self._log(f"[smooth] > 끝:   [{fN}]")

            self._log(
                f"[smooth] ✅ 완료 — {r['n_waypoints']} waypoint, "
                f"vel={self.spin_vel.value()}°/s acc={self.spin_acc.value()}°/s²")
            self._log(f"[smooth] > 저장: {r['out_path']}")

            self._set_status(
                f"평활화 완료 ({r['n_waypoints']} wp, max점프 {r['max_jump']}°)",
                "blue")
            self.btn_play.setEnabled(True)
            self._refresh_action_list()
            self._load_action(self.current_action)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Smooth 실패", str(e))
            self._log(f"[smooth] ❌ 실패: {e}")

    # ─── 버튼: Play / Home / E-STOP ──────────────────────
    def _on_play(self):
        if not self.current_action:
            return
        smooth_p = RECORDS_DIR / self.current_action / "smooth.json"
        if not smooth_p.exists():
            QtWidgets.QMessageBox.warning(self, "Play", "smooth.json 없음")
            return
        ans = QtWidgets.QMessageBox.question(
            self, "Play 확인",
            "재생을 시작합니다. 작업 공간이 안전한가요?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        if ans != QtWidgets.QMessageBox.Yes:
            return
        self._set_status("재생 중", "green")
        self.btn_play.setEnabled(False)
        # interrupt worker 의 paused 플래그 초기화
        self.interrupt.reset()
        self.request_play.emit(
            str(smooth_p), self.spin_vel.value(), self.spin_acc.value())

    def _on_play_started(self, n: int):
        self.progress.setMaximum(n)
        self.progress.setValue(0)
        self._log(f"[play] amovesj 시작 ({n} waypoint)")
        # 재생 제어 버튼 활성화
        self.btn_pause.setEnabled(True)
        self.btn_resume.setEnabled(False)
        self.btn_abort.setEnabled(True)
        # play 중엔 jog 금지
        self._set_jog_enabled(False, "재생 중")
        # play 시작 시각 기록 — _on_play_finished 에서 measured_duration 학습용
        self._play_t0 = time.monotonic()
        self._play_vel_used = float(self.spin_vel.value()) * \
            (max(1, self.slider_ops.value()) / 100.0)
        # 그리퍼 events timeline 발사 thread 시작 (해당 액션에 events 있고 활성 시)
        self._start_gripper_play_timeline()

    def _on_play_finished(self, rc: int, msg: str):
        # ⚠ measured_duration 은 진입 즉시 측정 — _stop_gripper_play_timeline 의
        #   grace 1.5s 가 측정에 포함되지 않도록.
        if hasattr(self, "_play_t0"):
            self._measured_play_dur = time.monotonic() - self._play_t0
        else:
            self._measured_play_dur = 0.0
        self.progress.setValue(self.progress.maximum())
        self._log(f"[play] 종료: {msg}")
        if rc == 0:
            color, status = "green", "재생 완료"
        elif rc == -2:
            color, status = "purple", "중단됨"
        else:
            color, status = "red", f"rc={rc}"
        self._set_status(status, color)
        self.btn_play.setEnabled(True)
        # 재생 제어 버튼 모두 비활성화
        self.btn_pause.setEnabled(False)
        self.btn_resume.setEnabled(False)
        self.btn_abort.setEnabled(False)
        self.interrupt.reset()
        # 그리퍼 events thread 정리 — 정상 종료면 grace 길게, abort 면 즉시
        is_abort = (rc != 0)
        self._stop_gripper_play_timeline(abort=is_abort)
        # 1) final state 보장 — 마지막 event width 강제 발사 (events 미발사 케이스 안전망)
        self._ensure_gripper_final_state()
        # 2) measured_duration 학습 → 다음 play 부터 정확
        # (진입 즉시 측정한 _measured_play_dur 사용 — grace 영향 제외)
        if rc == 0 and self._measured_play_dur > 0:
            self._update_measured_duration(self._measured_play_dur)
        # play 종료 → jog 가능 여부 재평가
        self._refresh_jog_enabled()
        # * Home 버튼이 abort 를 트리거한 경우 → 이제 홈 복귀 시작
        if self._home_after_abort:
            self._home_after_abort = False
            self._log("[home] abort 완료 — 홈 복귀 시작")
            # _busy 가 False 가 될 때까지 살짝 대기 후 emit
            QtCore.QTimer.singleShot(200, lambda: self.request_home.emit())

    # ─── 재생 제어 (Pause / Resume / Abort) ───────────────
    def _on_pause(self):
        self._log("[ctrl] pause 요청")
        self.request_pause.emit()

    def _on_resume(self):
        self._log("[ctrl] resume 요청")
        self.request_resume.emit()

    def _on_abort(self):
        self._log("[ctrl] abort 요청")
        self.request_abort.emit()

    def _on_paused_changed(self, paused: bool):
        # paused → Pause 비활성, Resume 활성
        # playing → Pause 활성, Resume 비활성
        self.btn_pause.setEnabled(not paused)
        self.btn_resume.setEnabled(paused)
        if paused:
            self._set_status("일시정지", "orange")
        else:
            # abort 직후에도 paused_changed(False) 가 emit 됨 — abort 핸들러가
            # 별도로 status 갱신하므로 여기서는 abort 가 아닐 때만 "재생 중" 으로 복원
            if not self.dsr._abort_requested:
                self._set_status("재생 중", "green")

    def _on_aborted(self):
        # abort 시 폴링 루프가 곧 종료되어 _on_play_finished 가 자동 호출됨.
        # 여기서는 Pause/Resume 즉시 비활성화로 UI 응답성 확보.
        self.btn_pause.setEnabled(False)
        self.btn_resume.setEnabled(False)
        self.btn_abort.setEnabled(False)
        self._set_status("중단 중...", "purple")

    # ─── 속도 컨트롤 ─────────────────────────────────────
    def _apply_speed_preset(self, vel: float, acc: float, name: str):
        self.spin_vel.setValue(vel)
        self.spin_acc.setValue(acc)
        self._log(f"[speed] preset '{name}' → vel={vel}°/s, acc={acc}°/s²")

    def _suggest_speed_from_raw(self):
        """
        raw.json 의 인접 샘플 속도 분포 → 90% 분위수를 추천 vel 로.
        평균이 아닌 90% 분위수: 정지/천천히 구간이 평균을 끌어내려
        체감보다 너무 느린 값이 나오는 것을 방지.
        """
        if not self.current_action:
            QtWidgets.QMessageBox.information(
                self, "추천", "액션을 먼저 로드하세요")
            return
        raw_p = RECORDS_DIR / self.current_action / "raw.json"
        if not raw_p.exists():
            QtWidgets.QMessageBox.warning(
                self, "추천", f"raw.json 없음:\n{raw_p}")
            return
        try:
            raw = json.loads(raw_p.read_text())
            ts = np.asarray(raw["timestamps_ms"], dtype=float)
            js = np.asarray(raw["joints_deg"], dtype=float)
            if len(ts) < 5:
                self._log("[speed] 샘플 부족 — 추천 불가")
                return
            dts = np.diff(ts) / 1000.0
            dts = np.where(dts > 1e-4, dts, 1e-3)
            dqs = np.linalg.norm(np.diff(js, axis=0), axis=1)
            speeds = dqs / dts                          # [°/s] 순간
            peak = float(np.percentile(speeds, 90))
            mean = float(speeds.mean())
            # 추천: peak 의 1.2배 (안전 마진 + 컨트롤러 가감속 손실 보정)
            sug_vel = max(15.0, min(360.0, round(peak * 1.2 / 5) * 5))
            sug_acc = min(720.0, sug_vel * 2.0)
            self.spin_vel.setValue(sug_vel)
            self.spin_acc.setValue(sug_acc)
            self._log(
                f"[speed] 시연 분석: mean={mean:.1f}°/s, "
                f"p90={peak:.1f}°/s")
            self._log(
                f"[speed] 추천 적용 → vel={sug_vel:.0f}°/s, "
                f"acc={sug_acc:.0f}°/s² (peak×1.2)")
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "추천 실패", str(e))
            self._log(f"[speed] 추천 실패: {e}")

    def _on_ops_slider_changed(self, value: int):
        # 드래그 중에는 라벨/실효속도만 갱신 (서비스 호출 X)
        self.lbl_ops.setText(f"{value}%")
        self._update_effective_label()

    def _on_ops_slider_released(self):
        # 손 뗀 순간 컨트롤러에 반영 (서비스 호출 1회)
        v = self.slider_ops.value()
        self.request_set_ops_speed.emit(v)

    def _update_effective_label(self):
        v = self.spin_vel.value()
        o = self.slider_ops.value()
        eff = v * o / 100.0
        self.lbl_effective.setText(
            f"실효 vel = {v:.0f} × {o}% = {eff:.1f} °/s")

    def _on_home(self):
        # * 재생/일시정지 중이면 먼저 abort 한 뒤 polling 루프가 끝날 때까지
        #   대기 → play_finished 시그널을 받으면 그때 home 발송.
        playing_or_paused = (
            self.dsr._busy
            or self.interrupt.is_paused()
            or self.btn_pause.isEnabled()
            or self.btn_resume.isEnabled()
        )
        if playing_or_paused:
            self._log("[home] 재생/일시정지 중 — abort 후 홈 복귀")
            self._home_after_abort = True   # play_finished 핸들러에서 사용
            self.request_abort.emit()
            return
        self._log("[home] 홈 복귀 요청")
        self.request_home.emit()

    def _on_estop(self):
        """E-STOP — 모든 동작 즉시 취소 + HOME 자동 복귀.

        흐름:
          1. record 진행 중이면 즉시 stop (buffer 보존)
          2. DSR stop(DR_SSTOP) 발사 — 모든 모션 즉시 정지
          3. 그리퍼 timeline thread abort (jog/timeline 모두 중단)
          4. _on_home 호출 — playing/paused 상태 자동 abort 후 HOME 복귀
        """
        self._log("[estop] 🛑 비상 정지 + HOME 복귀")
        # 1) record 진행 중이면 stop
        if self.recording:
            try:
                self._on_stop()
            except Exception as e:
                self._log(f"[estop] record stop 실패: {e}")
        # 2) DSR stop 즉시 발사 (모든 모션 즉시 멈춤)
        self.request_estop.emit()
        # 3) 그리퍼 timeline 즉시 abort
        try:
            self._stop_gripper_play_timeline(abort=True)
        except Exception as e:
            self._log(f"[estop] grip timeline abort 실패: {e}")
        # 4) HOME 자동 복귀 — DSR stop settle 시간 (~0.8s) 후
        # _on_home 이 playing/paused 상태 자동 처리 (abort 후 home)
        QtCore.QTimer.singleShot(800, self._on_home)

    # ─── DualSense (PS5 컨트롤러) ─────────────────────────
    def _set_dualsense_pill(self, active: bool, connected: bool, name: str):
        """상태바 🎮 pill 색상/텍스트 갱신.
        - 비활성   : 회색
        - 활성/미연결: 주황 ("🎮 DualSense 검색 중…")
        - 활성/연결 : 청색 ("🎮 DualSense ✓ name")
        """
        if not active:
            text, bg, fg = "🎮 DualSense OFF", "#eef0f3", "#6b7280"
        elif not connected:
            text, bg, fg = "🎮 DualSense 검색 중…", "#fef3c7", "#92400e"
        else:
            short = (name or "")[:24]
            text, bg, fg = f"🎮 DualSense ✓ {short}", "#dbeafe", "#1e40af"
        self.sb_dualsense.setText(text)
        self.sb_dualsense.setStyleSheet(
            f"padding:2px 8px; border-radius:8px; "
            f"background:{bg}; color:{fg}; "
            f"font-size:9pt; font-weight:600;")

    @QtCore.pyqtSlot(bool)
    def _on_toggle_dualsense(self, checked: bool):
        self._dualsense_active = checked
        _virtual = (self._launch_mode == "virtual")
        if checked:
            self._log("[ds] 활성화 — pygame 폴링 시작")
            # 초기 선택 joint 동기 + 강조
            self.dualsense.set_selected_joint(self._selected_joint)
            self._highlight_joint_row(self._selected_joint)
            self.dualsense.start()
            self._set_dualsense_pill(active=True, connected=False, name="")
            # 그리퍼도 자동 활성화 (한 묶음 컨트롤 — 메뉴에서 별도 토글 제거)
            # virtual 모드에서는 Modbus 연결 불가 → 그리퍼 건너뜀
            if _virtual:
                self._log("[ds] virtual 모드 — 그리퍼 연결 건너뜀")
            else:
                self._on_toggle_gripper(True)
        else:
            self._log("[ds] 비활성화")
            self.dualsense.stop()
            self._highlight_joint_row(-1)   # 강조 해제
            self._set_dualsense_pill(active=False, connected=False, name="")
            # 그리퍼도 같이 비활성화
            if not _virtual:
                self._on_toggle_gripper(False)

    @QtCore.pyqtSlot(bool, str)
    def _on_dualsense_connection_changed(self, ok: bool, name: str):
        self._set_dualsense_pill(
            active=self._dualsense_active, connected=ok, name=name)
        if ok:
            self._set_status(f"DualSense 연결: {name}", "blue")
        else:
            if self._dualsense_active:
                self._set_status("DualSense 연결 끊김 — 재검색", "orange")

    @QtCore.pyqtSlot(int)
    def _on_joint_selection_changed(self, idx: int):
        if not (0 <= idx < 6):
            return
        self._selected_joint = idx
        self._highlight_joint_row(idx)

    # TCP 좌표 axis 라벨 (X/Y/Z mm, A/B/C deg)
    _TCP_AXIS_NAMES = ["X", "Y", "Z", "A", "B", "C"]
    _TCP_AXIS_UNITS = ["mm", "mm", "mm", "°", "°", "°"]

    @QtCore.pyqtSlot(str)
    def _on_dualsense_jog_mode_changed(self, mode: str):
        """Options 버튼: Joint ↔ TCP jog 모드 토글 → 패널 좌표 표시도 전환."""
        self._jog_display_mode = mode
        if not hasattr(self, "jog_status_label"):
            return
        if mode == 'tcp':
            # tag 라벨 변경 (J{i} → X/Y/Z/A/B/C)
            for i, tag in enumerate(self.axis_tags):
                tag.setText(
                    f"<b>{self._TCP_AXIS_NAMES[i]}</b>"
                    f"<span style='color:#9ca3af; font-size:8pt;'>"
                    f" {self._TCP_AXIS_UNITS[i]}</span>")
            # 좌표 값 placeholder + TCP 폴링 시작 (1Hz)
            for lab in self.joint_labels:
                lab.setText("…")
            self.request_posx.emit()                # 즉시 1회
            self._tcp_posx_timer.start(1000)        # 이후 1Hz
            self.jog_status_label.setText(
                "🌐 [TCP·BASE] 좌측 LX→±X / LY→±Y, 우측 RY→±Z (mm/s)")
            self.jog_status_label.setStyleSheet(
                "color:#065f46; font-size:9pt; padding:2px;"
                "background:#d1fae5; font-weight:600;")
            self._highlight_joint_row(-1)           # joint 강조 해제
        else:
            # tag 복구 (X/Y/Z → J1~J6)
            for i, tag in enumerate(self.axis_tags):
                tag.setText(f"<b>J{i+1}</b>")
            # TCP 폴링 정지 + joint 콜백 다시 갱신 (다음 _on_joint 가 즉시)
            self._tcp_posx_timer.stop()
            self.jog_status_label.setText(
                "🦾 [JOINT] 좌측 스틱 = joint 선택 / 우측 스틱 = jog")
            self.jog_status_label.setStyleSheet(
                "color:#1e40af; font-size:9pt; padding:2px;"
                "background:#dbeafe; font-weight:600;")
            self._highlight_joint_row(self._selected_joint)
        self._log(f"[ds] jog mode → {mode.upper()}")

    @QtCore.pyqtSlot(list)
    def _on_posx_received(self, posx: list):
        """DsrWorker.query_posx 결과 — TCP 좌표 6축 표시 (mode='tcp' 일 때만)."""
        if self._jog_display_mode != 'tcp':
            return
        for i, lab in enumerate(self.joint_labels):
            if i < len(posx):
                lab.setText(f"{posx[i]:>+8.2f}")

    @QtCore.pyqtSlot()
    def _on_dualsense_record_toggle(self):
        """Create 버튼: 녹화 안 중이면 시작, 중이면 정지. GUI 버튼과 동일 경로."""
        if self.recording:
            self._log("[ds] Create → Stop")
            self._on_stop()
        else:
            self._log("[ds] Create → Record")
            self._on_record()

    def _do_play_no_confirm(self):
        """confirm dialog 없이 즉시 Play. DualSense 핸들러 전용."""
        if not self.current_action:
            self._log("[ds] PLAY 무시 — 액션 미선택")
            return
        smooth_p = RECORDS_DIR / self.current_action / "smooth.json"
        if not smooth_p.exists():
            self._log("[ds] PLAY 무시 — smooth.json 없음")
            return
        if self.dsr._busy:
            self._log("[ds] PLAY 거부 — DSR 작업 진행 중")
            return
        self._set_status("재생 중", "green")
        self.btn_play.setEnabled(False)
        self.interrupt.reset()
        self.request_play.emit(
            str(smooth_p), self.spin_vel.value(), self.spin_acc.value())

    # 표준 버튼 분류 (accept/reject fallback 결정용)
    _POSITIVE_STD_BUTTONS = frozenset({
        QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.Ok,
        QtWidgets.QMessageBox.YesToAll, QtWidgets.QMessageBox.Save,
        QtWidgets.QMessageBox.Apply, QtWidgets.QMessageBox.Open,
    })
    _NEGATIVE_STD_BUTTONS = frozenset({
        QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Cancel,
        QtWidgets.QMessageBox.NoToAll, QtWidgets.QMessageBox.Close,
        QtWidgets.QMessageBox.Abort, QtWidgets.QMessageBox.Discard,
    })

    def _try_dialog_button(self, *std_buttons) -> bool:
        """현재 활성 모달의 표준 버튼 클릭 시도. 3 단계 fallback.
        QMetaObject.invokeMethod 로 메인 thread queued 실행 — modal exec_() 의
        nested event loop 가 잘 처리하도록 보장.
        진단 로그는 DualSense 디버그 모드 (Ctrl+Shift+D) 일 때만."""
        modal = QtWidgets.QApplication.activeModalWidget()
        if modal is None:
            return False
        cls = type(modal).__name__
        verbose = (hasattr(self, "act_dualsense_debug")
                   and self.act_dualsense_debug.isChecked())
        # 1) QMessageBox 직접
        if isinstance(modal, QtWidgets.QMessageBox):
            for std in std_buttons:
                btn = modal.button(std)
                if btn is not None:
                    QtCore.QMetaObject.invokeMethod(
                        btn, "click", QtCore.Qt.QueuedConnection)
                    if verbose:
                        self._log(f"[ds][diag] modal={cls} → button(std={int(std)}).click")
                    return True
        # 2) 내부 QDialogButtonBox (모든 인스턴스 검색)
        # ⚠ QDialogButtonBox.button() 은 QDialogButtonBox.StandardButton enum 만 받음.
        #   QMessageBox.StandardButton 과 값은 같지만 type 다름 → 명시 변환 필요.
        for bb in modal.findChildren(QtWidgets.QDialogButtonBox):
            for std in std_buttons:
                try:
                    bb_std = QtWidgets.QDialogButtonBox.StandardButton(int(std))
                    btn = bb.button(bb_std)
                except (ValueError, TypeError):
                    btn = None
                if btn is not None:
                    QtCore.QMetaObject.invokeMethod(
                        btn, "click", QtCore.Qt.QueuedConnection)
                    if verbose:
                        self._log(f"[ds][diag] modal={cls} → bb.button(std={int(std)}).click")
                    return True
        # 3) 일반 QDialog accept/reject — std_buttons 의 첫 번째 부호로 결정
        if isinstance(modal, QtWidgets.QDialog) and std_buttons:
            first = std_buttons[0]
            if first in self._POSITIVE_STD_BUTTONS:
                QtCore.QMetaObject.invokeMethod(
                    modal, "accept", QtCore.Qt.QueuedConnection)
                if verbose:
                    self._log(f"[ds][diag] modal={cls} → accept() (queued)")
                return True
            if first in self._NEGATIVE_STD_BUTTONS:
                QtCore.QMetaObject.invokeMethod(
                    modal, "reject", QtCore.Qt.QueuedConnection)
                if verbose:
                    self._log(f"[ds][diag] modal={cls} → reject() (queued)")
                return True
        if verbose:
            self._log(f"[ds][diag] modal={cls} — fallback 매칭 실패")
        return False

    @QtCore.pyqtSlot()
    def _on_dualsense_smooth_play(self):
        """○ : 다이얼로그 떠있으면 Yes/Ok 클릭, 아니면 SMOOTH+PLAY 흐름."""
        if self._try_dialog_button(
                QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.Ok):
            self._log("[ds] ○ → 다이얼로그 Yes/Ok")
            return
        if not self.current_action:
            self._log("[ds] ○ 무시 — 액션 미선택")
            return
        action_dir = RECORDS_DIR / self.current_action
        smooth_p = action_dir / "smooth.json"
        raw_p    = action_dir / "raw.json"
        if smooth_p.exists():
            self._log("[ds] ○ → smooth 존재, 즉시 PLAY")
            self._do_play_no_confirm()
            return
        if not raw_p.exists():
            self._log("[ds] ○ 무시 — raw.json 도 없음 (먼저 RECORD)")
            return
        self._log("[ds] ○ → SMOOTH 실행 후 자동 PLAY")
        self._on_smooth()  # 동기 호출 — 완료 시 smooth.json 생성됨
        if smooth_p.exists():
            self._do_play_no_confirm()
        else:
            self._log("[ds] ○ — SMOOTH 실패 → PLAY 취소")

    @QtCore.pyqtSlot()
    def _on_dualsense_pause_resume_toggle(self):
        """× short-press: 다이얼로그 떠있으면 No/Cancel, 아니면 Pause/Resume."""
        if self._try_dialog_button(
                QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.Cancel):
            self._log("[ds] × → 다이얼로그 No/Cancel")
            return
        if self.interrupt.is_paused():
            self._log("[ds] × → Resume")
            self.request_resume.emit()
        else:
            self._log("[ds] × → Pause")
            self.request_pause.emit()

    @QtCore.pyqtSlot()
    def _on_dualsense_estop(self):
        """× long-press (≥1s): 비상 정지 + HOME. 다이얼로그 떠있으면 무시."""
        if QtWidgets.QApplication.activeModalWidget() is not None:
            self._log("[ds] × long-press 무시 — 다이얼로그 활성")
            return
        self._log("[ds] × long-press → E-STOP + HOME")
        self._on_estop()   # 통합 핸들러 — stop + home

    @QtCore.pyqtSlot()
    def _on_dualsense_home(self):
        """△ : Home 위치로 복귀. 재생/일시정지 중이면 abort 후 자동 home."""
        self._log("[ds] △ → HOME")
        self._on_home()

    @QtCore.pyqtSlot()
    def _on_dualsense_new_profile(self):
        """Create long-press(≥2s): 자동 이름으로 새 프로파일 즉시 시작."""
        if self.recording:
            self._log("[ds] Create long → 진행 중인 record 먼저 STOP")
            self._on_stop()
        name = f"action_{datetime.now():%Y%m%d_%H%M%S}"
        self._log(f"[ds] Create long → 새 프로파일 '{name}' 시작")
        self._start_record(name)

    @QtCore.pyqtSlot()
    def _on_dualsense_mode_toggle(self):
        """L3+R3: MANUAL ↔ AUTONOMOUS 전환. 기존 메뉴 핸들러 재사용."""
        self._log("[ds] L3+R3 → 모드 전환")
        self._on_toggle_mode()

    @QtCore.pyqtSlot(bool)
    def _on_toggle_dualsense_debug(self, checked: bool):
        """DualSense 자세 입력 로그 (verbose) 토글."""
        try:
            self.dualsense.set_verbose(checked)
        except Exception as e:
            self._log(f"[ds][debug] verbose 토글 실패: {e}")

    @QtCore.pyqtSlot()
    def _on_show_dualsense_cheatsheet(self):
        """DualSense 매핑 cheat sheet 다이얼로그."""
        msg = (
            "<h3>🎮 DualSense × ros2_move_recoder 매핑</h3>"
            "<table cellpadding='4' style='font-size:10pt;'>"
            "<tr><th align='left'>버튼</th><th align='left'>동작</th></tr>"
            "<tr><td><b>Create</b> (short)</td><td>Record on/off 토글</td></tr>"
            "<tr><td><b>Create</b> (≥2s hold)</td><td>새 프로파일 자동 생성</td></tr>"
            "<tr><td><b>○</b></td><td>Smooth + Play (다이얼로그 있으면 → Yes/Ok)</td></tr>"
            "<tr><td><b>×</b> (short)</td><td>Pause ↔ Resume (다이얼로그 있으면 → No/Cancel)</td></tr>"
            "<tr><td><b>×</b> (≥1s hold)</td><td>🛑 E-STOP — 모든 동작 취소 + HOME 복귀 (다이얼로그 중엔 무시)</td></tr>"
            "<tr><td><b>△</b></td><td>Home 위치 복귀</td></tr>"
            "<tr><td><b>□</b></td><td>그리퍼 Open ↔ Close 토글 (활성 시)</td></tr>"
            "<tr><td><b>Options</b></td><td>🦾 JOINT ↔ 🌐 TCP jog 모드 토글</td></tr>"
            "<tr><td colspan='2'><i>[JOINT 모드]</i></td></tr>"
            "<tr><td><b>좌측 스틱 ↑/→</b></td><td>다음 joint (J1→J2→…→J6→J1 wrap)</td></tr>"
            "<tr><td><b>좌측 스틱 ↓/←</b></td><td>이전 joint</td></tr>"
            "<tr><td><b>우측 스틱</b></td><td>선택 joint jog (°/s, 응답곡선 \\|s\\|^1.5)</td></tr>"
            "<tr><td colspan='2'><i>[TCP·BASE 모드]</i></td></tr>"
            "<tr><td><b>좌측 스틱 ←/→</b></td><td>−X / +X jog (mm/s)</td></tr>"
            "<tr><td><b>좌측 스틱 ↑/↓</b></td><td>+Y / −Y jog</td></tr>"
            "<tr><td><b>우측 스틱 ↑/↓</b></td><td>+Z / −Z jog</td></tr>"
            "<tr><td><b>D-Pad</b></td><td>(예약, 미사용)</td></tr>"
            "<tr><td><b>L2 트리거</b> (hold)</td><td>속도 −1°/s 지속 감소 (10Hz)</td></tr>"
            "<tr><td><b>R2 트리거</b> (hold)</td><td>속도 +1°/s 지속 증가 (10Hz)</td></tr>"
            "<tr><td><b>L3 + R3</b></td><td>MANUAL ↔ AUTONOMOUS 토글</td></tr>"
            "</table>"
            "<p style='color:#6b7280; font-size:9pt;'>"
            "디버그 모드(Ctrl+Shift+D) 켜면 모든 입력이 로그에 표시됩니다.<br>"
            "Button 인덱스 안 맞으면 dualsense_worker.py 의 BTN_* 상수 조정.</p>")
        QtWidgets.QMessageBox.information(
            self, "DualSense 매핑", msg)

    def _check_modal_active(self):
        """100ms 주기 — modal 활성 변화 시 DualSense 워커에 통보 → jog 일시정지."""
        is_modal = (QtWidgets.QApplication.activeModalWidget() is not None)
        if is_modal != self._last_modal_state:
            self._last_modal_state = is_modal
            try:
                self.dualsense.set_modal_active(is_modal)
            except Exception:
                pass

    # ─── 그리퍼 (OnRobot RG2/RG6) ─────────────────────────
    def _set_gripper_pill(self, active: bool, connected: bool,
                          width_mm: float | None):
        if not active:
            text, bg, fg = "🦾 Gripper OFF", "#eef0f3", "#6b7280"
        elif not connected:
            text, bg, fg = "🦾 Gripper 연결 실패", "#fee2e2", "#991b1b"
        else:
            w_str = f"{width_mm:.1f}mm" if width_mm is not None else "?"
            text, bg, fg = f"🦾 Gripper ✓ {w_str}", "#dbeafe", "#1e40af"
        self.sb_gripper.setText(text)
        self.sb_gripper.setStyleSheet(
            f"padding:2px 8px; border-radius:8px; "
            f"background:{bg}; color:{fg}; "
            f"font-size:9pt; font-weight:600;")

    def _set_gripper_ui_enabled(self, enabled: bool, reason: str = ""):
        if not hasattr(self, "_grip_widgets"):
            return
        for w in self._grip_widgets:
            w.setEnabled(enabled)
        if hasattr(self, "lbl_grip_status"):
            if enabled:
                self.lbl_grip_status.setText(
                    "준비 — Open/Close 버튼 또는 □ 버튼 (DualSense)")
                self.lbl_grip_status.setStyleSheet(
                    "color:#065f46; font-size:9pt; padding:2px;")
            else:
                self.lbl_grip_status.setText(f"비활성 — {reason}")
                self.lbl_grip_status.setStyleSheet(
                    "color:#92400e; font-size:9pt; padding:2px;")

    @QtCore.pyqtSlot(bool)
    def _on_toggle_gripper(self, checked: bool):
        self._gripper_active = checked
        if checked:
            self._log("[grip] 활성화 — 워커 시작")
            self.gripper.start()
            self._set_gripper_pill(active=True, connected=False, width_mm=None)
            self._set_gripper_ui_enabled(True)
        else:
            self._log("[grip] 비활성화")
            self.gripper.stop()
            self._gripper_width_mm = None
            self._set_gripper_pill(active=False, connected=False, width_mm=None)
            self._set_gripper_ui_enabled(False, "비활성")
            self.lbl_grip_width.setText("— mm")

    @QtCore.pyqtSlot(bool, str)
    def _on_gripper_connection_changed(self, ok: bool, msg: str):
        self._set_gripper_pill(
            active=self._gripper_active, connected=ok,
            width_mm=self._gripper_width_mm)
        if not ok and self._gripper_active:
            self._set_status(f"그리퍼 연결 실패: {msg}", "red")

    @QtCore.pyqtSlot(float)
    def _on_gripper_width_changed(self, width_mm: float):
        self._gripper_width_mm = float(width_mm)
        self.lbl_grip_width.setText(f"{width_mm:.1f} mm")
        self._set_gripper_pill(
            active=self._gripper_active, connected=True, width_mm=width_mm)

    def _start_gripper_play_timeline(self):
        """play 시작 시 호출. smooth.json 의 gripper_events 를 amovesj 진행 시간
        추정으로 timeline 에 따라 발사.

        진행 시간 추정: arc-length / (vel × ops_ratio) × 1.10 (가속 마진).
        amovesj ramp-up 영향 ~10-20%, gripper events 가 보통 few-second 단위라
        충분한 정확도. 실 duration > est 면 마지막 events 가 미발사 가능 →
        _stop_gripper_play_timeline 의 grace period 로 보완.
        """
        if not self._gripper_active:
            self._log("[grip][play] skip — 그리퍼 비활성")
            return
        if not self.gripper.is_connected():
            self._log("[grip][play] skip — 그리퍼 미연결")
            return
        if not self.current_action:
            self._log("[grip][play] skip — 액션 미선택")
            return
        smooth_p = RECORDS_DIR / self.current_action / "smooth.json"
        if not smooth_p.exists():
            self._log("[grip][play] skip — smooth.json 없음")
            return
        try:
            sm = json.loads(smooth_p.read_text())
        except Exception as e:
            self._log(f"[grip][play] smooth.json 로드 실패: {e}")
            return
        events = sm.get("gripper_events") or []
        if not events:
            self._log("[grip][play] skip — gripper_events 없음")
            return

        try:
            wps = np.asarray(sm["waypoints_deg"], dtype=float)
            vel = float(sm.get("vel", 60.0))
            ops_ratio = max(1, self.slider_ops.value()) / 100.0
            current_play_vel = vel * ops_ratio
            if len(wps) < 2 or current_play_vel <= 0:
                self._log(
                    f"[grip][play] skip — 부적절 (n_wps={len(wps)}, "
                    f"play_vel={current_play_vel})")
                return
            # ── est_duration 결정 (우선순위) ──
            # 1) measured_duration_sec × (measured_play_vel / current_play_vel)
            #    ← 이전 play 의 실 측정값. 가장 정확. 단 vel scaling 보정.
            # 2) record_duration_sec × (smooth_vel / play_vel)
            #    ← record 시간 기반. 첫 play 시 사용 (실 amovesj 보다 보통 +30%).
            # 3) arc-length / vel × margin
            #    ← old action (record_duration 없음) fallback
            measured_dur = float(sm.get("measured_duration_sec", 0.0))
            measured_vel = float(sm.get("measured_at_play_vel", 0.0))
            record_dur = float(sm.get("record_duration_sec", 0.0))
            smooth_vel = float(sm.get("vel", 60.0))
            if measured_dur > 0 and measured_vel > 0:
                est_duration = measured_dur * (measured_vel / current_play_vel)
                est_source = (
                    f"measured={measured_dur:.2f}s × "
                    f"({measured_vel:.0f}/{current_play_vel:.0f})")
            elif record_dur > 0 and smooth_vel > 0:
                est_duration = record_dur * (smooth_vel / current_play_vel)
                est_source = (
                    f"record_dur={record_dur:.2f}s × "
                    f"({smooth_vel:.0f}/{current_play_vel:.0f}) "
                    f"— 첫 play, 다음 play 부터 measured 학습됨")
            else:
                dists = np.linalg.norm(np.diff(wps, axis=0), axis=1)
                total_dist = float(dists.sum())
                est_duration = total_dist / current_play_vel * 1.1
                est_source = (
                    f"arc={total_dist:.1f}°/{current_play_vel:.1f}°/s × 1.10 "
                    f"(record_duration 없음 — old action)")
        except Exception as e:
            self._log(f"[grip][play] duration 추정 실패: {e}")
            return

        self._log(
            f"[grip][play] timeline 시작 — events={len(events)} "
            f"est_duration={est_duration:.2f}s [{est_source}]")
        for i, ev in enumerate(events):
            self._log(
                f"[grip][play]   event#{i+1}: t_norm={ev['t_norm']:.3f} "
                f"({ev['t_norm']*est_duration:.2f}s) → "
                f"width={ev['width_mm']:.1f}mm")

        self._grip_play_stop.clear()
        events_sorted = sorted(events, key=lambda e: e["t_norm"])
        log_sig = self._thread_log_sig
        gripper = self.gripper   # daemon thread 에서 안전 capture
        stop_evt = self._grip_play_stop

        # 그리퍼 모터 1회 동작 최소 시간 (OnRobot RG2 ~2s)
        GRIP_MIN_GAP_S = 2.0

        # timeline 시작 시점 gripper 상태 로그
        self._log(
            f"[grip][play]   gripper 상태: connected={gripper.is_connected()}, "
            f"last_width={gripper.last_width()}, "
            f"last_state={gripper.last_state()}")

        def _run_timeline():
            try:
                log_sig.emit(
                    f"[grip][play] thread 진입 ({len(events_sorted)} events)")
                t0 = time.monotonic()
                last_emit_t = -1e9
                for i, ev in enumerate(events_sorted):
                    target_t = ev["t_norm"] * est_duration
                    target_t = max(target_t, last_emit_t + GRIP_MIN_GAP_S)
                    # target_t 까지 대기 (50ms 주기로 stop 체크)
                    while not stop_evt.is_set():
                        elapsed = time.monotonic() - t0
                        remaining = target_t - elapsed
                        if remaining <= 0:
                            break
                        time.sleep(min(0.05, remaining))
                    if stop_evt.is_set():
                        log_sig.emit(
                            f"[grip][play] 중단 — event#{i+1}/"
                            f"{len(events_sorted)} 미발사")
                        return
                    actual_t = time.monotonic() - t0
                    # 발사 직전 gripper 상태 검증
                    if not gripper.is_connected():
                        log_sig.emit(
                            f"[grip][play] ⚠ event#{i+1} skip — gripper 미연결")
                        last_emit_t = actual_t
                        continue
                    try:
                        w = float(ev["width_mm"])
                        # ⚠ kind 따라 □ 와 동일 함수 호출 — raw width 정확도와 무관.
                        #   fallback: kind 없는 옛 events 는 width 임계로 분류.
                        kind = ev.get("kind")
                        if kind is None:
                            kind = ('close' if w <= 20.0
                                    else ('open' if w >= 50.0 else 'move'))
                        log_sig.emit(
                            f"[grip][play] ▶ event#{i+1}/{len(events_sorted)} "
                            f"@ {actual_t:.2f}s → {kind} ({w:.1f}mm) "
                            f"[was {gripper.last_width()}mm]")
                        if kind == 'close':
                            gripper.close()        # □ 와 동일
                        elif kind == 'open':
                            gripper.open()         # □ 와 동일
                        else:
                            gripper.move(w)
                        last_emit_t = actual_t
                    except Exception as e:
                        log_sig.emit(
                            f"[grip][play] 명령 실패 ev#{i+1}: "
                            f"{type(e).__name__}: {e}")
                log_sig.emit("[grip][play] timeline 종료 (모든 events 처리됨)")
            except Exception as e:
                import traceback
                log_sig.emit(
                    f"[grip][play] thread 예외: {type(e).__name__}: {e}")
                for line in traceback.format_exc().strip().split("\n")[-4:]:
                    log_sig.emit(f"[grip][play]   {line}")

        self._grip_play_thread = threading.Thread(
            target=_run_timeline, daemon=True, name="gripper-play-timeline")
        self._grip_play_thread.start()

    def _ensure_gripper_final_state(self):
        """play 종료 직후 호출. smooth.json 의 마지막 event width 를 강제로
        발사 — timeline 이 모든 events 미발사 (실 모션 시간 < est 일 때) 케이스
        에서도 그리퍼가 record 의 final state 로 끝나도록 보장."""
        if not self._gripper_active or not self.gripper.is_connected():
            return
        if not self.current_action:
            return
        smooth_p = RECORDS_DIR / self.current_action / "smooth.json"
        if not smooth_p.exists():
            return
        try:
            sm = json.loads(smooth_p.read_text())
        except Exception:
            return
        events = sm.get("gripper_events") or []
        if not events:
            return
        final = events[-1]
        try:
            self._log(
                f"[grip][play] final state 보장 → "
                f"move({float(final['width_mm']):.1f}mm)")
            self.gripper.move(float(final["width_mm"]))
        except Exception as e:
            self._log(f"[grip][play] final state 발사 실패: {e}")

    def _update_measured_duration(self, measured: float):
        """play 종료 후 실 측정 시간을 smooth.json 에 학습 저장.
        다음 play 부터 record_duration 보다 우선 사용 → 정확한 events timing."""
        if measured <= 0.5 or not self.current_action:
            return
        smooth_p = RECORDS_DIR / self.current_action / "smooth.json"
        if not smooth_p.exists():
            return
        try:
            sm = json.loads(smooth_p.read_text())
            sm["measured_duration_sec"] = round(measured, 3)
            sm["measured_at_play_vel"] = round(
                getattr(self, "_play_vel_used", float(sm.get("vel", 60.0))), 2)
            smooth_p.write_text(json.dumps(sm, indent=2))
            self._log(
                f"[grip][play] measured_duration 학습 → "
                f"{measured:.2f}s @ play_vel={sm['measured_at_play_vel']}°/s")
        except Exception as e:
            self._log(f"[grip][play] measured_duration 저장 실패: {e}")

    def _stop_gripper_play_timeline(self, abort: bool = False):
        """play 종료 시 호출.
        - 정상 종료: grace 8s — events 모두 발사 + 모터 시간 (1.5s × N) 보장
        - abort: 즉시 stop (위험 회피 우선)
        """
        if self._grip_play_thread is None or \
           not self._grip_play_thread.is_alive():
            self._grip_play_thread = None
            return
        if abort:
            self._log("[grip][play] abort — 즉시 중단")
            self._grip_play_stop.set()
            self._grip_play_thread.join(timeout=0.5)
        else:
            # 정상 종료 — events 자연 완료 대기 (모터 시간 포함)
            self._grip_play_thread.join(timeout=8.0)
            if self._grip_play_thread.is_alive():
                self._log("[grip][play] grace(8s) 만료 — 강제 중단")
                self._grip_play_stop.set()
                self._grip_play_thread.join(timeout=0.5)
        self._grip_play_thread = None

    @QtCore.pyqtSlot()
    def _on_dualsense_gripper_toggle(self):
        """□ : 그리퍼 활성 시 마지막 명령 기준 open ↔ close 토글."""
        if not self._gripper_active:
            self._log("[ds] □ 무시 — 그리퍼 비활성")
            return
        if not self.gripper.is_connected():
            self._log("[ds] □ 무시 — 그리퍼 미연결")
            return
        # 마지막 상태 기준 반전. 'closed' / 'open' / 'mid' / 'unknown'
        st = self.gripper.last_state()
        if st == "closed":
            self._log("[ds] □ → Gripper Open")
            self.gripper.open()
        else:
            # 'open' / 'mid' / 'unknown' 모두 → close
            self._log("[ds] □ → Gripper Close")
            self.gripper.close()

    @QtCore.pyqtSlot(float)
    def _on_dualsense_speed_step(self, delta: float):
        """L2(−)/R2(+): 현재 선택 joint 의 mini-jog 속도 ± step."""
        idx = self._selected_joint
        if not (0 <= idx < 6):
            return
        sb = self.jog_vel_inputs[idx]
        new_vel = max(sb.minimum(), min(sb.maximum(), sb.value() + delta))
        if abs(new_vel - sb.value()) > 1e-6:
            sb.setValue(new_vel)
            sign = "+" if delta > 0 else "−"
            self._log(
                f"[ds] {sign}L{'/R'[delta>0]}2 → J{idx+1} 속도 "
                f"{new_vel:.0f} °/s")

    def _highlight_joint_row(self, idx: int):
        """통합 패널에서 J{idx} 행을 강조.
        idx == -1 → 모든 행 강조 해제 (DualSense 비활성 시).
        비선택 행 색은 테마 토큰 사용 — 다크/라이트 모두 가독성 보장.
        """
        if not hasattr(self, "joint_labels"):
            return
        # 테마 토큰 — dark 면 밝은 텍스트, light 면 짙은 텍스트
        unsel_fg = self._theme.get("text", "#1a1d23")
        for i, lab in enumerate(self.joint_labels):
            if i == idx:
                # 선택 — 청색 배경 + 굵은 좌측 보더
                lab.setStyleSheet(
                    "font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
                    "font-size: 14pt; color:#1e40af; font-weight:700;"
                    "letter-spacing: 0.5px;"
                    "background:#dbeafe; padding:2px 6px;"
                    "border-left:3px solid #1e40af;")
            else:
                # 비선택 — 테마 메인 텍스트 색 (대비 보장)
                lab.setStyleSheet(
                    "font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
                    f"font-size: 14pt; color:{unsel_fg}; font-weight:500;"
                    "letter-spacing: 0.5px;")

    # ─── Mini-Jog ────────────────────────────────────────
    def _on_jog_pressed(self, joint_idx: int, direction: int):
        """버튼 누름 → jog 시작. direction = +1 / −1."""
        if not (0 <= joint_idx < 6):
            return
        # 위젯 자체가 disabled 면 pressed 시그널이 안 오지만 안전 가드
        if not self.jog_minus_btns[joint_idx].isEnabled():
            return
        vel = float(self.jog_vel_inputs[joint_idx].value()) * direction
        # axis = joint_idx (0~5), ref = 0 (joint 모드는 ignored)
        self.request_jog.emit(joint_idx, 0, vel)

    def _on_jog_released(self):
        """버튼 뗌 → 즉시 정지. dispatcher 가 jog(0,0,0) 발사."""
        # 박스가 disabled 여도 정지 명령은 보내야 안전 (잔여 모션 차단)
        self.request_stop_jog.emit()

    def _set_jog_enabled(self, enabled: bool, reason: str = ""):
        """mini-jog 위젯만 enable 토글 + 상태 텍스트 갱신.
        좌표 라벨은 항상 표시. disable 로 전환할 땐 stop_jog 자동 발사."""
        if not hasattr(self, "_jog_widgets"):
            return
        # 첫 토글이거나 상태 변화가 있을 때만 동작
        was_enabled = (
            self._jog_widgets[0].isEnabled() if self._jog_widgets else False)
        for w in self._jog_widgets:
            w.setEnabled(enabled)
        if was_enabled and not enabled:
            self.request_stop_jog.emit()
        if hasattr(self, "jog_status_label"):
            if enabled:
                self.jog_status_label.setText(
                    "Mini-Jog 준비 — 누르는 동안 jog, 떼면 정지")
                self.jog_status_label.setStyleSheet(
                    "color:#065f46; font-size:9pt; padding:2px;")
            else:
                self.jog_status_label.setText(f"Mini-Jog 비활성 — {reason}")
                self.jog_status_label.setStyleSheet(
                    "color:#92400e; font-size:9pt; padding:2px;")

    def _refresh_jog_enabled(self):
        """mode / busy 상태를 보고 mini-jog 활성 여부를 갱신."""
        if self.dsr._busy:
            self._set_jog_enabled(False, "DSR 작업 진행 중")
            return
        if self.robot_mode != MODE_AUTONOMOUS:
            self._set_jog_enabled(False, "AUTONOMOUS 모드 필요")
            return
        self._set_jog_enabled(True)

    # ─── 유틸 ───────────────────────────────────────────
    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_view.appendPlainText(f"[{ts}] {msg}")

    def _log_debug(self, msg: str):
        """진단/디버깅 전용 로그 — debug_enabled True 일 때만 출력."""
        if self.debug_enabled:
            self._log(msg)

    def _on_debug_toggle(self, checked: bool):
        self.debug_enabled = checked
        self._update_debug_btn_style()
        self._log(f"[debug] 진단 로그 {'ON' if checked else 'OFF'}")

    def _update_debug_btn_style(self):
        if not hasattr(self, "btn_debug_toggle"):
            return
        t = self._theme
        on = bool(self.btn_debug_toggle.isChecked())
        if on:
            self.btn_debug_toggle.setText("DEBUG  ●  ON")
            self.btn_debug_toggle.setStyleSheet(
                "QPushButton {"
                f"  padding:2px 10px; background:{t['debug_on_bg']};"
                f"  color:{t['debug_on_fg']};"
                f"  border:1px solid {t['debug_on_border']}; border-radius:9px;"
                "  font-size:8pt; font-weight:600; letter-spacing:0.5px; }"
                f"QPushButton:hover {{ background:{t['debug_on_bg']}; }}"
            )
        else:
            self.btn_debug_toggle.setText("DEBUG  ○  OFF")
            self.btn_debug_toggle.setStyleSheet(
                "QPushButton {"
                f"  padding:2px 10px; background:{t['debug_off_bg']};"
                f"  color:{t['debug_off_fg']};"
                f"  border:1px solid {t['debug_off_border']}; border-radius:9px;"
                "  font-size:8pt; font-weight:600; letter-spacing:0.5px; }"
                f"QPushButton:hover {{ background:{t['btn_neutral_hover']}; }}"
            )

    def _set_status(self, text: str, color: str):
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            f"color: {color}; font-size: 10pt; font-weight: 600;")

    # ─── 테마 시스템 ──────────────────────────────────────
    def _apply_theme(self, name: str):
        """테마 변경 — qApp stylesheet + 모든 인라인 스타일 + 모든 _mkbtn 버튼 재스타일."""
        if name not in THEMES:
            return
        self._theme_name = name
        self._theme = THEMES[name]
        self._btn_variants = btn_variants_for_theme(self._theme)

        # 1) 글로벌 stylesheet
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.setStyleSheet(build_stylesheet(self._theme))

        # 2) _mkbtn 으로 만든 모든 버튼 (variant 태그가 있는 것) 재스타일
        for b in self.findChildren(QtWidgets.QPushButton):
            v = getattr(b, "_variant", None)
            if v is not None:
                self._apply_btn_style(b, v)

        # 3) 인라인 스타일 헬퍼들 재호출
        self._restyle_static_widgets()
        self._update_debug_btn_style()
        if hasattr(self, "robot_mode"):
            self._on_mode_updated_style_only(self.robot_mode)

        # 4) 메뉴 액션 체크 갱신
        if hasattr(self, "act_theme_light") and hasattr(self, "act_theme_dark"):
            self.act_theme_light.setChecked(name == "light")
            self.act_theme_dark.setChecked(name == "dark")

    def _restyle_static_widgets(self):
        """build_ui 에서 setStyleSheet 으로 박은 위젯들 (헤더 라벨/조인트/슬라이더/preset 등)
        을 현재 테마 토큰으로 다시 스타일."""
        t = self._theme
        # 상단 액션 라벨
        self.action_label.setStyleSheet(
            f"font-size: 13pt; font-weight: 600; color: {t['text']}; letter-spacing: 0.2px;")
        # 모드 배지 (현재 모드 색은 _on_mode_updated_style_only 에서 처리, 기본만 여기)
        self.status_label.setStyleSheet(
            f"color: {t['text_muted']}; font-size: 10pt; font-weight: 500;")

        # 조인트 라벨
        if hasattr(self, "joint_labels"):
            for lab in self.joint_labels:
                lab.setStyleSheet(
                    "font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
                    f"font-size: 14pt; color:{t['text']}; font-weight:500;"
                    "letter-spacing: 0.5px;")
        if hasattr(self, "sample_count_label"):
            self.sample_count_label.setStyleSheet(
                "font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
                f"font-size: 12pt; color:{t['danger']}; font-weight:600;")

        # joint/sample 태그 (각 폼 행 좌측의 'J1', '샘플' 등)
        # → QFormLayout 으로 추가된 라벨이라 직접 추적이 까다로움. 동적으로 form 의 row count
        #   를 돌면서 left-side label 을 갱신해도 되지만, build_stylesheet 의 QLabel 글로벌
        #   스타일이 이미 색을 잡으므로 추가 처리 생략.

        # OPS 라벨 (퍼센트)
        if hasattr(self, "lbl_ops"):
            self.lbl_ops.setStyleSheet(
                "font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
                f"font-weight:600; color:{t['accent']}; font-size:10pt;")
        if hasattr(self, "lbl_effective"):
            self.lbl_effective.setStyleSheet(
                f"color:{t['text_dim']}; "
                "font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
                "font-size:9pt; padding:2px 4px;")

        # Preset (Slow/Normal/Fast/Max) 4개
        preset_style = (
            f"QPushButton {{ background:{t['btn_neutral_bg']}; color:{t['text_secondary']};"
            f"  border:1px solid {t['border_strong']}; border-radius:5px;"
            f"  padding:6px 10px; font-size:9pt; font-weight:600;"
            f"  letter-spacing:0.3px; }}"
            f"QPushButton:hover {{ background:{t['btn_neutral_hover']};"
            f"  border-color:{t['border_strong']}; }}"
            f"QPushButton:pressed {{ background:{t['preset_pressed_bg']};"
            f"  color:{t['accent']}; border-color:{t['accent']}; }}"
        )
        if hasattr(self, "_preset_buttons"):
            for pb in self._preset_buttons:
                pb.setStyleSheet(preset_style)

        # 자동 추천 버튼
        if hasattr(self, "btn_auto_speed"):
            self.btn_auto_speed.setStyleSheet(
                f"QPushButton {{ padding:8px 10px; background:{t['btn_neutral_bg']};"
                f"  color:{t['accent']}; border:1px solid {t['accent']}; border-radius:5px;"
                f"  font-size:9.5pt; font-weight:600; letter-spacing:0.3px; }}"
                f"QPushButton:hover {{ background:{t['accent_bg']};"
                f"  border-color:{t['accent']}; }}"
                f"QPushButton:pressed {{ background:{t['accent_bg']}; }}"
            )

        # 로그 뷰
        if hasattr(self, "log_view"):
            self.log_view.setStyleSheet(
                "QPlainTextEdit {"
                "  font-family: 'JetBrains Mono', 'Menlo', 'Consolas', monospace;"
                "  font-size: 9pt; line-height: 1.4;"
                f"  background: {t['log_bg']}; color: {t['log_text']};"
                f"  border: 1px solid {t['log_border']}; border-radius: 6px;"
                f"  padding: 8px; selection-background-color: {t['accent']}; }}"
            )

    def _on_mode_updated_style_only(self, mode):
        """현재 모드에 맞춰 mode_label 배지 색만 다시 적용 (텍스트 변경 없음)."""
        if not hasattr(self, "mode_label"):
            return
        t = self._theme
        name = MODE_NAMES.get(mode, None)
        bg, fg = {
            "MANUAL":     (t["warning_bg"], t["warning_fg"]),
            "AUTONOMOUS": (t["success_bg"], t["success_fg"]),
        }.get(name, (t["mode_default_bg"], t["mode_default_fg"]))
        self.mode_label.setStyleSheet(
            f"padding:4px 10px; border-radius:10px;"
            f"background:{bg}; color:{fg};"
            f"font-size:9pt; font-weight:600; letter-spacing:0.6px;"
        )

    def _on_theme_change(self, name: str):
        self._apply_theme(name)
        self._log(f"[theme] {name} 모드로 전환")

    # ─── 종료 처리 ──────────────────────────────────────
    def closeEvent(self, event):
        self._log("[exit] 종료 중...")
        try:
            self.mode_timer.stop()
            self._hz_timer.stop()
            self._modal_timer.stop()
            self._tcp_posx_timer.stop()
        except Exception:
            pass
        try:
            self.dualsense.stop()
        except Exception:
            pass
        try:
            self._stop_gripper_play_timeline()
            self.gripper.stop()
        except Exception:
            pass
        try:
            self.worker_thread.quit()
            self.worker_thread.wait(1000)
        except Exception:
            pass
        try:
            self.interrupt_thread.quit()
            self.interrupt_thread.wait(1000)
        except Exception:
            pass
        try:
            self.js.stop()
            self.js.wait(2000)
        except Exception:
            pass
        try:
            self.ros.stop()
            self.ros.wait(2000)
        except Exception:
            pass
        self.bringup.shutdown()
        event.accept()


# ════════════════════════════════════════════════════════════════════
# 테마 토큰 — light / dark 두 벌. _build_stylesheet / _btn_variants /
# 인라인 setStyleSheet 헬퍼들이 이 토큰을 참조해 동적 생성한다.
THEMES = {
    "light": {
        "bg":              "#f5f6f8",
        "surface":         "#ffffff",
        "surface_alt":     "#f3f4f6",
        "border":          "#e3e6ea",
        "border_strong":   "#d8dce2",
        "text":            "#1a1d23",
        "text_secondary":  "#4a525e",
        "text_muted":      "#6b7280",
        "text_dim":        "#8b95a3",
        "accent":          "#4f46e5",
        "accent_hover":    "#4338ca",
        "accent_press":    "#3730a3",
        "accent_bg":       "#eef2ff",
        "success":         "#10b981",
        "success_bg":      "#d1fae5",
        "success_fg":      "#065f46",
        "success_border":  "#10b981",
        "warning":         "#f59e0b",
        "warning_bg":      "#fef3c7",
        "warning_fg":      "#92400e",
        "warning_border":  "#f59e0b",
        "danger":          "#dc2626",
        "danger_hover":    "#b91c1c",
        "danger_press":    "#991b1b",
        "log_bg":          "#15171c",
        "log_text":        "#d4d7dc",
        "log_border":      "#24272d",
        "btn_neutral_bg":      "#ffffff",
        "btn_neutral_hover":   "#f3f4f6",
        "btn_neutral_press":   "#e5e7eb",
        "btn_disabled_bg":     "#f5f6f8",
        "btn_disabled_text":   "#aab1bb",
        "btn_disabled_border": "#e3e6ea",
        "preset_pressed_bg":   "#eef2ff",
        "menubar_bg":      "#ffffff",
        "debug_off_bg":    "#ffffff",
        "debug_off_fg":    "#6b7280",
        "debug_off_border": "#d8dce2",
        "debug_on_bg":     "#dcfce7",
        "debug_on_fg":     "#15803d",
        "debug_on_border": "#86efac",
        "mode_default_bg": "#eef0f3",
        "mode_default_fg": "#5b6470",
    },
    "dark": {
        "bg":              "#13151a",
        "surface":         "#1c1f26",
        "surface_alt":     "#252932",
        "border":          "#2d3139",
        "border_strong":   "#3a3f4a",
        "text":            "#e4e6eb",
        "text_secondary":  "#b3b9c4",
        "text_muted":      "#8b95a3",
        "text_dim":        "#6b7280",
        "accent":          "#7c7df9",
        "accent_hover":    "#9395fb",
        "accent_press":    "#5d5fe5",
        "accent_bg":       "#2a2d52",
        "success":         "#34d399",
        "success_bg":      "#0f3a2a",
        "success_fg":      "#6ee7b7",
        "success_border":  "#10b981",
        "warning":         "#fbbf24",
        "warning_bg":      "#3a2e0c",
        "warning_fg":      "#fcd34d",
        "warning_border":  "#f59e0b",
        "danger":          "#f87171",
        "danger_hover":    "#ef4444",
        "danger_press":    "#dc2626",
        "log_bg":          "#0d0f13",
        "log_text":        "#d4d7dc",
        "log_border":      "#1c1f26",
        "btn_neutral_bg":      "#1c1f26",
        "btn_neutral_hover":   "#252932",
        "btn_neutral_press":   "#2d3139",
        "btn_disabled_bg":     "#1c1f26",
        "btn_disabled_text":   "#5b6470",
        "btn_disabled_border": "#2d3139",
        "preset_pressed_bg":   "#2a2d52",
        "menubar_bg":      "#1c1f26",
        "debug_off_bg":    "#1c1f26",
        "debug_off_fg":    "#8b95a3",
        "debug_off_border": "#2d3139",
        "debug_on_bg":     "#0f3a2a",
        "debug_on_fg":     "#6ee7b7",
        "debug_on_border": "#10b981",
        "mode_default_bg": "#252932",
        "mode_default_fg": "#b3b9c4",
    },
}


def build_stylesheet(t: dict) -> str:
    """토큰 dict 로부터 QApplication-wide stylesheet 문자열 생성."""
    return f"""
* {{ font-family: "Inter", "Pretendard", "Noto Sans CJK KR", "Segoe UI", sans-serif; }}

QMainWindow, QWidget {{ background: {t['bg']}; color: {t['text']}; }}

QGroupBox {{
    border: 1px solid {t['border']};
    border-radius: 8px;
    background: {t['surface']};
    margin-top: 14px;
    padding: 8px 10px 10px 10px;
    font-weight: 600;
    color: {t['text_secondary']};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 0 6px;
    color: {t['text_muted']};
    font-size: 9pt;
    font-weight: 600;
    letter-spacing: 0.4px;
}}

QLabel {{ color: {t['text']}; }}

QListWidget {{
    background: {t['surface']};
    border: 1px solid {t['border']};
    border-radius: 6px;
    padding: 4px;
    font-size: 10pt;
    color: {t['text']};
}}
QListWidget::item {{ padding: 6px 8px; border-radius: 4px; }}
QListWidget::item:selected {{ background: {t['accent_bg']}; color: {t['text']}; }}
QListWidget::item:hover {{ background: {t['surface_alt']}; }}

QPushButton {{
    background: {t['btn_neutral_bg']};
    color: {t['text']};
    border: 1px solid {t['border_strong']};
    border-radius: 6px;
    padding: 6px 12px;
    font-size: 10pt;
}}
QPushButton:hover {{ background: {t['btn_neutral_hover']}; }}
QPushButton:pressed {{ background: {t['btn_neutral_press']}; }}
QPushButton:disabled {{
    background: {t['btn_disabled_bg']};
    color: {t['btn_disabled_text']};
    border-color: {t['btn_disabled_border']};
}}

QSpinBox, QDoubleSpinBox, QLineEdit {{
    background: {t['surface']};
    border: 1px solid {t['border_strong']};
    border-radius: 5px;
    padding: 4px 6px;
    color: {t['text']};
    selection-background-color: {t['accent']};
    selection-color: white;
}}
QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {{
    border: 1px solid {t['accent']};
}}

QSlider::groove:horizontal {{
    height: 4px; background: {t['border']}; border-radius: 2px;
}}
QSlider::sub-page:horizontal {{ background: {t['accent']}; border-radius: 2px; }}
QSlider::handle:horizontal {{
    background: {t['surface']}; border: 1px solid {t['accent']};
    width: 14px; height: 14px; margin: -6px 0; border-radius: 7px;
}}

QProgressBar {{
    background: {t['surface']};
    border: 1px solid {t['border']};
    border-radius: 6px;
    text-align: center;
    color: {t['text_secondary']};
    font-size: 9pt;
}}
QProgressBar::chunk {{
    background: {t['accent']};
    border-radius: 5px;
}}

QPlainTextEdit {{
    background: {t['log_bg']};
    color: {t['log_text']};
    border: 1px solid {t['log_border']};
    border-radius: 6px;
    padding: 6px 8px;
    selection-background-color: {t['accent']};
}}

QStatusBar {{ background: {t['bg']}; border-top: 1px solid {t['border']}; color: {t['text_muted']}; font-size: 9pt; }}
QMenuBar {{ background: {t['menubar_bg']}; border-bottom: 1px solid {t['border']}; color: {t['text']}; }}
QMenuBar::item:selected {{ background: {t['accent_bg']}; }}
QMenu {{ background: {t['surface']}; border: 1px solid {t['border']}; color: {t['text']}; }}
QMenu::item:selected {{ background: {t['accent_bg']}; }}
QMenu::separator {{ height: 1px; background: {t['border']}; margin: 4px 8px; }}
"""


def btn_variants_for_theme(t: dict) -> dict:
    """테마 토큰에서 _mkbtn 의 BTN_VARIANTS 사전을 동적 생성."""
    return {
        "primary": {
            "bg": t["accent"], "fg": "#ffffff", "border": t["accent"],
            "hover_bg": t["accent_hover"], "hover_border": t["accent_hover"],
            "press_bg": t["accent_press"],
        },
        "neutral": {
            "bg": t["btn_neutral_bg"], "fg": t["text"], "border": t["border_strong"],
            "hover_bg": t["btn_neutral_hover"], "hover_border": t["border_strong"],
            "press_bg": t["btn_neutral_press"],
        },
        "danger": {
            "bg": t["danger"], "fg": "#ffffff", "border": t["danger"],
            "hover_bg": t["danger_hover"], "hover_border": t["danger_hover"],
            "press_bg": t["danger_press"],
        },
    }


def main():
    RECORDS_DIR.mkdir(parents=True, exist_ok=True)
    app = QtWidgets.QApplication(sys.argv)
    # 디폴트 = dark. MainWindow._apply_theme 가 모든 위젯의 인라인 스타일도 함께 갱신.
    app.setStyleSheet(build_stylesheet(THEMES["dark"]))
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
