"""
ROBO CHEF — Coordinate Service (JointStates-first) + Safety Bridge

설계
  - /dsr01/joint_states 를 전용 CallbackGroup(MultiThreadedExecutor 상의 별 스레드)
    으로 "항상" 수신해서 관절 위치(deg)·속도(deg/s)·토크(effort) 를 캐시한다.
  - TCP 좌표는 joint_states 를 입력으로 robot_state_publisher 가 계산한
    /tf(base_link → link_6) 를 주기적으로 룩업해 얻는다 (별도 FK 구현 없음).
  - Firebase Realtime DB /telemetry/robot_status 에 주기적으로 snapshot 을 write 해
    admin.thatshoon.com 에서 실시간 구독·표시 가능하게 한다.
  - 터미널에도 joint/velocity/torque 를 실시간 표시한다. TCP 는 서브노드 백엔드가 FK 로 계산.
  - SafetyBridge — /commands/robot (Firebase) 를 listen 해서 pause/resume/stop/
    reset_safe_stop/servo_on 명령을 raw rclpy 로 DSR 컨트롤러에 전달. 결과는
    /commands/robot_ack 에 write. 좌표 스트림과는 전용 CallbackGroup 으로 격리된다.
  - /telemetry/robot_state 에 get_robot_state() 결과를 2Hz 로 발행 (LED/복구용).

외부 인터페이스
  ROS 서비스 : /robo_chef/get_coords  (std_srvs/srv/Trigger) — 요청 시 최신 JSON 반환
  ROS 토픽   : /robo_chef/coords      (std_msgs/String, 2Hz)
  Firebase   : /telemetry/robot_status (5Hz set)
               /telemetry/robot_state  (2Hz set — robot_state code + 이름)
               /commands/robot         (listen — pause/resume/stop/... 수신)
               /commands/robot_ack     (set — 명령 실행 결과)

주의 (CLAUDE.md dsr-skill)
  - DSR_ROBOT2 는 import 하지 않는다 (g_node 고정/executor 충돌 회피).
  - 크로스-네트워크 환경에서 TF 룩업 주기는 10Hz 이하로 유지한다.
  - SafetyBridge 는 raw rclpy 서비스 클라이언트만 사용 (DSR_ROBOT2 래퍼 없음).
"""

import json
import math
import os
import queue
import sys
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

# ── 설정 ─────────────────────────────────────────────────────────
ROBOT_MODEL = "m0609"

COORD_SERVICE  = "/robo_chef/get_coords"
COORD_TOPIC    = "/robo_chef/coords"
TOPIC_JOINTS   = "/dsr01/joint_states"
# TCP 좌표는 서브노드 백엔드(fk_worker)가 FK 로 계산해서 Firebase 에 기록.
# 여기서는 joint/velocity/effort 와 로봇 state 만 publish.

PUBLISH_HZ       = 2      # ROS 토픽 /robo_chef/coords
DISPLAY_HZ       = 10     # 터미널 디스플레이
FIREBASE_PUSH_HZ = 5      # Firebase set 주기
STALE_SEC        = 2.0

FIREBASE_DB_URL_DEFAULT = "https://robochef-5d9b6-default-rtdb.asia-southeast1.firebasedatabase.app"
FIREBASE_DB_URL       = os.environ.get("FIREBASE_DB_URL", FIREBASE_DB_URL_DEFAULT)
FIREBASE_TELEMETRY    = "telemetry/robot_status"
FIREBASE_ROBOT_STATE  = "robot_state"          # canonical 상태 노드 (Rokey_1 backend 공유)
ROBOT_STATE_PUSH_HZ   = 2                      # /robot_state merge-update 주기
# FIREBASE_CRED_PATH 환경변수 우선. 없으면 후보 경로 중 첫 번째 존재 파일 사용.
FIREBASE_CRED_CANDIDATES = [
    os.path.expanduser("~/.config/cobot1/firebase-key.json"),
    os.path.expanduser("~/cobot_ws/src/cobot1/main_side/robo_chef/config/serviceAccountKey.json"),
    os.path.expanduser("~/cobot_ws/src/robo_chef/config/serviceAccountKey.json"),
    os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "..", "..", "robo_chef", "config",
                                  "serviceAccountKey.json")),
]


def _resolve_firebase_cred() -> str | None:
    env = os.environ.get("FIREBASE_CRED_PATH")
    if env and os.path.isfile(env):
        return env
    return next((p for p in FIREBASE_CRED_CANDIDATES if os.path.isfile(p)), None)

# ── SafetyBridge 설정 ────────────────────────────────────────────
ROBOT_ID           = "dsr01"
FB_CMD_PATH        = "commands/robot"
FB_ACK_PATH        = "commands/robot_ack"
FB_STATE_PATH      = "telemetry/robot_state"
STATE_POLL_HZ      = 2       # get_robot_state → /telemetry/robot_state 주기
SVC_WAIT_TIMEOUT   = 3.0     # service server 연결 대기 (s)
SVC_CALL_TIMEOUT   = 6.0     # service 호출 응답 대기 (s)

ROBOT_STATE_MAP = {
    0:  "INITIALIZING", 1:  "STANDBY", 2:  "MOVING",
    3:  "SAFE_OFF",     4:  "TEACHING", 5: "SAFE_STOP",
    6:  "EMERGENCY_STOP", 7: "HOMMING", 8: "RECOVERY",
    9:  "SAFE_STOP2",   10: "SAFE_OFF2",
    15: "NOT_READY",
}

# ── 캐시 ─────────────────────────────────────────────────────────
_cache = {
    "joint":     [0.0] * 6,   # deg
    "velocity":  [0.0] * 6,   # deg/s
    "effort":    [0.0] * 6,   # torque (Nm/raw)
    "timestamp": "",
    "req_count": 0,
}
_lock        = threading.Lock()
_last_joint  = 0.0
_joint_ready = False

_node = None


def _log(level, msg):
    if _node is None:
        sys.stderr.write(f"[{level.upper()}] {msg}\n")
        sys.stderr.flush()
        return
    logger = _node.get_logger()
    if   level == "warn":  logger.warn(msg)
    elif level == "error": logger.error(msg)
    elif level == "debug": logger.debug(msg)
    else:                  logger.info(msg)


# ── ANSI ─────────────────────────────────────────────────────────
_CYAN, _GREEN, _YELLOW, _MAG, _DIM, _BOLD, _RESET = (
    "\033[36m", "\033[32m", "\033[33m", "\033[35m", "\033[2m", "\033[1m", "\033[0m"
)
_DISPLAY_HEIGHT = 11
_display_ready  = False


# ════════════════════════════════════════════════════════════════
#  /dsr01/joint_states 구독 — 항상 수신 (전용 콜백 그룹)
# ════════════════════════════════════════════════════════════════

def _on_joint_states(msg: JointState):
    """rad→deg 로 변환하고 속도·토크까지 캐시. name 순서가 섞여있을 수 있으므로 정렬."""
    global _last_joint, _joint_ready

    if len(msg.position) < 6 or len(msg.name) < 6:
        return

    # name 별 인덱스 매핑 (joint_1..joint_6)
    try:
        idx = [msg.name.index(f"joint_{i}") for i in range(1, 7)]
    except ValueError as e:
        _log("warn", f"[joint_states] 누락된 조인트 이름: {e}")
        return

    def _clean(v):
        """NaN/Inf → 0.0 (Firebase JSON 직렬화는 NaN 거부)."""
        if v is None or not math.isfinite(v):
            return 0.0
        return float(v)

    def _safe(seq, i, default=0.0):
        return _clean(seq[i]) if i < len(seq) else default

    joint_deg    = [round(math.degrees(_safe(msg.position, k)), 3) for k in idx]
    velocity_deg = [round(math.degrees(_safe(msg.velocity, k)), 3) for k in idx]
    effort_val   = [round(_safe(msg.effort, k), 3)                for k in idx]

    ts = datetime.now(timezone.utc).isoformat()
    with _lock:
        _cache["joint"]     = joint_deg
        _cache["velocity"]  = velocity_deg
        _cache["effort"]    = effort_val
        _cache["timestamp"] = ts
    _last_joint = time.monotonic()
    if not _joint_ready:
        _joint_ready = True
        _log("info", f"[joint_states] 첫 수신: joint={joint_deg}")


# ════════════════════════════════════════════════════════════════
#  서비스 핸들러
# ════════════════════════════════════════════════════════════════

def _snapshot():
    with _lock:
        return {
            "joint":     list(_cache["joint"]),
            "velocity":  list(_cache["velocity"]),
            "effort":    list(_cache["effort"]),
            "timestamp": _cache["timestamp"],
        }


def _handle_get_coords(request, response):
    try:
        with _lock:
            _cache["req_count"] += 1
        response.success = True
        response.message = json.dumps(_snapshot(), ensure_ascii=False)
    except Exception as e:
        _log("error", f"[service] 핸들러 예외: {e}")
        response.success = False
        response.message = json.dumps({"error": str(e)})
    return response


# ════════════════════════════════════════════════════════════════
#  Firebase 업로더 — admin.thatshoon.com 실시간 피드
# ════════════════════════════════════════════════════════════════

class _FirebasePublisher:
    """별 스레드에서 Firebase RTDB 에 write.
    - /telemetry/robot_status : FIREBASE_PUSH_HZ 주기 raw snapshot (velocity/effort 포함)
    - /robot_state             : ROBOT_STATE_PUSH_HZ 주기 merge-update (canonical 상태)
                                joint_positions + last_updated 만 쓴다. 다른 필드
                                (robot_status, gripper_status, mode, current_step...)는
                                다른 writer(SafetyBridge, Rokey_1 backend)가 관리.
    """

    def __init__(self):
        self._ref = None
        self._robot_state_ref = None
        self._alive = False

    def start(self):
        try:
            import firebase_admin
            from firebase_admin import credentials, db
        except ImportError:
            _log("warn", "[firebase] firebase_admin 미설치 — pip install firebase-admin")
            return

        cred_path = _resolve_firebase_cred()
        if cred_path is None:
            _log("warn",
                 "[firebase] credential 파일 없음 — FIREBASE_CRED_PATH 환경변수 또는 "
                 f"후보: {FIREBASE_CRED_CANDIDATES}")
            return

        try:
            if not firebase_admin._apps:
                cred = credentials.Certificate(cred_path)
                firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
                _log("info", f"[firebase] 초기화 완료 (cred={cred_path})")
            self._ref             = db.reference(FIREBASE_TELEMETRY)
            self._robot_state_ref = db.reference(FIREBASE_ROBOT_STATE)
        except Exception as e:
            _log("error", f"[firebase] 초기화 실패: {e}")
            return

        self._alive = True
        threading.Thread(target=self._loop,             daemon=True, name="firebase").start()
        threading.Thread(target=self._robot_state_loop, daemon=True, name="robot_state").start()
        _log("info",
             f"[firebase] 업로더 시작 — /{FIREBASE_TELEMETRY} @ {FIREBASE_PUSH_HZ}Hz, "
             f"/{FIREBASE_ROBOT_STATE} @ {ROBOT_STATE_PUSH_HZ}Hz")

    def _loop(self):
        interval = 1.0 / FIREBASE_PUSH_HZ

        def _sanitize(obj):
            if isinstance(obj, list):
                return [_sanitize(x) for x in obj]
            if isinstance(obj, dict):
                return {k: _sanitize(v) for k, v in obj.items()}
            if isinstance(obj, float) and not math.isfinite(obj):
                return 0.0   # NaN/Inf → 0 (Firebase JSON 거부)
            return obj

        err_count = 0
        while self._alive:
            t0 = time.monotonic()
            snap = _sanitize(_snapshot())
            snap["model"]        = ROBOT_MODEL
            snap["source"]       = "joint_states"
            snap["ingested_at"]  = datetime.now(timezone.utc).isoformat()
            try:
                self._ref.set(snap)
                if err_count:
                    _log("info", f"[firebase] 업로드 복구 (실패 {err_count}회 후)")
                err_count = 0
            except Exception as e:
                err_count += 1
                if err_count <= 3 or err_count in (10, 100):
                    _log("warn", f"[firebase] 업로드 실패(x{err_count}): {e}")
            time.sleep(max(0.0, interval - (time.monotonic() - t0)))

    def _robot_state_loop(self):
        """/robot_state 에 joint_positions 와 last_updated 만 merge-update.
        .set() 금지 — 다른 writer 필드(robot_status, mode, gripper_status 등) 보존.

        Ownership matrix (architecture.md §6 참고):
          - joint_positions ← coord_service (이 함수, 2 Hz)
          - mode            ← SafetyBridge   (이 파일 line 517, 명령 ack 시점)
          - tcp_position    ← fk_worker      (sub1_side/web/backend, 2 Hz)
        last_updated 는 마지막 writer 의 시각."""
        interval = 1.0 / ROBOT_STATE_PUSH_HZ
        err_count = 0
        while self._alive:
            t0 = time.monotonic()
            # joint 값이 아직 안 들어왔으면 스킵 (초기 0,0,0,0,0,0 을 덮어쓰지 않음)
            if _joint_ready:
                with _lock:
                    joint = list(_cache["joint"])
                # NaN/Inf 가드 (Firebase JSON 거부)
                joint = [0.0 if not math.isfinite(v) else float(v) for v in joint]
                patch = {
                    "joint_positions": joint,
                    "last_updated":    datetime.now(timezone.utc).isoformat(),
                }
                try:
                    self._robot_state_ref.update(patch)
                    if err_count:
                        _log("info", f"[robot_state] 업로드 복구 (실패 {err_count}회 후)")
                    err_count = 0
                except Exception as e:
                    err_count += 1
                    if err_count <= 3 or err_count in (10, 100):
                        _log("warn", f"[robot_state] 업로드 실패(x{err_count}): {e}")
            time.sleep(max(0.0, interval - (time.monotonic() - t0)))


# ════════════════════════════════════════════════════════════════
#  SafetyBridge — /commands/robot listen + raw rclpy 서비스 호출
# ════════════════════════════════════════════════════════════════

def _await_future(future, timeout=SVC_CALL_TIMEOUT):
    """MultiThreadedExecutor 에 붙은 노드의 service future 를 폴링 대기.
    (spin_until_future_complete 는 이미 spin 중인 executor 와 재진입 위험)"""
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    return future.result() if future.done() else None


class SafetyBridge:
    """Firebase 에서 받은 정지/재개/복구 명령을 DSR 컨트롤러에 raw rclpy 로 전달."""

    def __init__(self, node, robot_id=ROBOT_ID, callback_group=None):
        self.node     = node
        self.robot_id = robot_id
        self.cg       = callback_group or ReentrantCallbackGroup()
        self._cmd_q   = queue.Queue()
        self._seen_issued_at = None
        self._alive   = False

        self._cmd_ref         = None
        self._ack_ref         = None
        self._state_ref       = None
        self._robot_state_ref = None

        self._clients = {}            # cmd_name -> (client, factory)
        self._get_state_client = None

    # ── 시작/클라이언트 초기화 ───────────────────────────────────
    def start(self):
        # dsr_msgs2 import 는 시작 시점에 (모듈 import 시 ROS 서비스 정의 로드 비용 분산)
        try:
            from dsr_msgs2.srv import (
                MovePause, MoveResume, MoveStop,
                SetRobotControl, GetRobotState, ServoOff,
            )
        except ImportError as e:
            _log("warn", f"[safety] dsr_msgs2 import 실패 — 비활성화: {e}")
            return

        def _stop_req():
            r = MoveStop.Request(); r.stop_mode = 2; return r   # DR_SSTO (Soft Stop)
        def _ctrl_req(v):
            r = SetRobotControl.Request(); r.robot_control = int(v); return r
        def _servo_off_req(st=2):
            r = ServoOff.Request(); r.stop_type = int(st); return r

        specs = {
            # 모션 제어 — 동기 motion 진행 중이어도 stop 은 별 cb_group 으로 즉시 처리됨
            "pause":           (MovePause,       f"/{self.robot_id}/motion/move_pause",        MovePause.Request),
            "resume":          (MoveResume,      f"/{self.robot_id}/motion/move_resume",       MoveResume.Request),
            "stop":            (MoveStop,        f"/{self.robot_id}/motion/stop",              _stop_req),
            # 시스템 복구
            "reset_safe_stop": (SetRobotControl, f"/{self.robot_id}/system/set_robot_control", lambda: _ctrl_req(2)),
            "servo_on":        (SetRobotControl, f"/{self.robot_id}/system/set_robot_control", lambda: _ctrl_req(3)),
            "recovery_enter_safe_stop": (SetRobotControl, f"/{self.robot_id}/system/set_robot_control", lambda: _ctrl_req(4)),
            "recovery_enter_safe_off":  (SetRobotControl, f"/{self.robot_id}/system/set_robot_control", lambda: _ctrl_req(5)),
            "recovery_exit":            (SetRobotControl, f"/{self.robot_id}/system/set_robot_control", lambda: _ctrl_req(7)),
            # 서보 Off — 긴급 정지 후 수동 조작용
            "servo_off":       (ServoOff,        f"/{self.robot_id}/system/servo_off",         lambda: _servo_off_req(2)),   # STOP_TYPE_SLOW
            "servo_off_quick": (ServoOff,        f"/{self.robot_id}/system/servo_off",         lambda: _servo_off_req(1)),   # STOP_TYPE_QUICK
        }
        for name, (SrvType, endpoint, factory) in specs.items():
            cli = self.node.create_client(SrvType, endpoint, callback_group=self.cg)
            self._clients[name] = (cli, factory, endpoint)

        # 상태 조회 전용 클라이언트
        self._get_state_client = self.node.create_client(
            GetRobotState, f"/{self.robot_id}/system/get_robot_state",
            callback_group=self.cg,
        )

        # Firebase ref 초기화
        try:
            import firebase_admin
            from firebase_admin import db
            if not firebase_admin._apps:
                _log("warn", "[safety] firebase 미초기화 — publisher 먼저 start 필요")
                return
            self._cmd_ref         = db.reference(FB_CMD_PATH)
            self._ack_ref         = db.reference(FB_ACK_PATH)
            self._state_ref       = db.reference(FB_STATE_PATH)
            self._robot_state_ref = db.reference(FIREBASE_ROBOT_STATE)
        except ImportError:
            _log("warn", "[safety] firebase_admin 미설치 — Firebase 경유 명령 비활성")
            return

        # Firebase listener (별 스레드 자동)
        self._cmd_ref.listen(self._on_cmd_event)

        self._alive = True
        threading.Thread(target=self._worker_loop,      daemon=True, name="safety-worker").start()
        threading.Thread(target=self._state_poll_loop,  daemon=True, name="safety-state").start()

        _log("info",
             f"[safety] 활성 — listen /{FB_CMD_PATH}, ack /{FB_ACK_PATH}, "
             f"state /{FB_STATE_PATH} @ {STATE_POLL_HZ}Hz")

    # ── Firebase 이벤트 콜백 (별 스레드) ─────────────────────────
    def _on_cmd_event(self, event):
        data = event.data
        if not isinstance(data, dict):
            return
        issued_at = data.get("issued_at")
        cmd       = data.get("cmd")
        if not cmd:
            return
        # 같은 이벤트 중복 트리거 방지
        if issued_at is not None and issued_at == self._seen_issued_at:
            return
        self._seen_issued_at = issued_at
        self._cmd_q.put({"cmd": cmd, "issued_at": issued_at,
                         "payload": data.get("payload") or {}})
        _log("info", f"[safety] 명령 수신: cmd={cmd} issued_at={issued_at}")

    # ── Worker — 명령 순차 처리 ──────────────────────────────────
    def _worker_loop(self):
        while self._alive:
            try:
                item = self._cmd_q.get(timeout=0.5)
            except queue.Empty:
                continue
            cmd = item["cmd"]
            t0 = time.monotonic()
            ok, detail = self._execute(cmd)
            self._write_ack(item, ok, detail, time.monotonic() - t0)

    def _execute(self, cmd):
        spec = self._clients.get(cmd)
        if spec is None:
            return False, f"알 수 없는 cmd: {cmd}"
        cli, factory, endpoint = spec
        if not cli.wait_for_service(timeout_sec=SVC_WAIT_TIMEOUT):
            return False, f"서비스 대기 타임아웃: {endpoint}"
        try:
            req = factory()
            future = cli.call_async(req)
            result = _await_future(future, timeout=SVC_CALL_TIMEOUT)
            if result is None:
                return False, f"응답 타임아웃: {endpoint}"
            success = getattr(result, "success", True)
            # servo_on 은 브레이크 해제 대기 (약 3초)
            if cmd == "servo_on" and success:
                time.sleep(3.5)
            return bool(success), "ok" if success else "service returned success=false"
        except Exception as e:
            return False, f"예외: {e}"

    def _write_ack(self, item, ok, detail, elapsed):
        if self._ack_ref is None:
            return
        ack = {
            "cmd":          item["cmd"],
            "issued_at":    item["issued_at"],
            "ok":           bool(ok),
            "detail":       detail,
            "elapsed_sec":  round(elapsed, 3),
            "processed_at": datetime.now(timezone.utc).isoformat(),
        }
        try:
            self._ack_ref.set(ack)
        except Exception as e:
            _log("warn", f"[safety] ACK write 실패: {e}")
        level = "info" if ok else "warn"
        _log(level, f"[safety] 처리 완료: cmd={item['cmd']} ok={ok} "
                    f"({elapsed:.2f}s) detail={detail}")

    # ── 상태 폴링 → Firebase /telemetry/robot_state ──────────────
    def _state_poll_loop(self):
        if self._get_state_client is None or self._state_ref is None:
            return
        interval = 1.0 / STATE_POLL_HZ
        last_code, err_count = None, 0
        # 서비스 대기 (bringup 기동 직후면 아직 안 떠있을 수 있음)
        self._get_state_client.wait_for_service(timeout_sec=SVC_WAIT_TIMEOUT)
        while self._alive:
            t0 = time.monotonic()
            try:
                from dsr_msgs2.srv import GetRobotState
                future = self._get_state_client.call_async(GetRobotState.Request())
                result = _await_future(future, timeout=1.5)
                if result is not None:
                    code = int(result.robot_state)
                    name = ROBOT_STATE_MAP.get(code, f"UNKNOWN({code})")
                    now_iso = datetime.now(timezone.utc).isoformat()
                    legacy_payload = {
                        "code":       code,
                        "name":       name,
                        "updated_at": now_iso,
                    }
                    # /robot_state.mode 는 name 문자열(STANDBY/MOVING/...)로 merge.
                    # Ownership: SafetyBridge 가 mode 필드 단독 소유 (architecture.md §6).
                    robot_state_patch = {
                        "mode":         name,
                        "last_updated": now_iso,
                    }
                    try:
                        self._state_ref.set(legacy_payload)          # 구 경로 (backward compat)
                        if self._robot_state_ref is not None:
                            self._robot_state_ref.update(robot_state_patch)
                    except Exception as e:
                        err_count += 1
                        if err_count in (1, 10, 100):
                            _log("warn", f"[safety] state write 실패(x{err_count}): {e}")
                    else:
                        err_count = 0
                    if code != last_code:
                        _log("info", f"[safety] robot_state: {last_code} → {code} ({name})")
                        last_code = code
            except Exception as e:
                if err_count == 0:
                    _log("warn", f"[safety] state poll 예외: {e}")
                err_count += 1
            time.sleep(max(0.0, interval - (time.monotonic() - t0)))


# ════════════════════════════════════════════════════════════════
#  터미널 표시
# ════════════════════════════════════════════════════════════════

def _print_display(snap, ok, req_cnt):
    global _display_ready
    now = datetime.now().strftime("%H:%M:%S")
    sep = f"{_DIM}{'─' * 64}{_RESET}"

    def _fmt(label, v, unit, color, width=8):
        return f"{_DIM}{label}{_RESET} {color}{v:+{width}.2f}{_RESET}{_DIM}{unit}{_RESET}"

    if ok:
        labels_j   = ["J1", "J2", "J3", "J4", "J5", "J6"]

        jnt = snap["joint"]
        vel = snap["velocity"]
        trq = snap["effort"]

        l_j1   = f"  {_CYAN}Joint    {_RESET} " + "  ".join(_fmt(labels_j[i],   jnt[i],     "°",  _YELLOW) for i in range(3))
        l_j2   = f"  {_CYAN}         {_RESET} " + "  ".join(_fmt(labels_j[i],   jnt[i],     "°",  _YELLOW) for i in range(3, 6))
        l_v1   = f"  {_CYAN}Velocity {_RESET} " + "  ".join(_fmt(labels_j[i],   vel[i],   "°/s",  _MAG)   for i in range(3))
        l_v2   = f"  {_CYAN}         {_RESET} " + "  ".join(_fmt(labels_j[i],   vel[i],   "°/s",  _MAG)   for i in range(3, 6))
        l_t1   = f"  {_CYAN}Torque   {_RESET} " + "  ".join(_fmt(labels_j[i],   trq[i],    "Nm",  _CYAN)  for i in range(3))
        l_t2   = f"  {_CYAN}         {_RESET} " + "  ".join(_fmt(labels_j[i],   trq[i],    "Nm",  _CYAN)  for i in range(3, 6))
        l_top  = f"  {_DIM}TCP 는 서브노드 fk_worker 가 계산 (여기는 joint/velocity/torque 만 publish){_RESET}"
    else:
        l_top  = f"  {_YELLOW}⚠ 데이터 미수신 — bringup 상태 및 /dsr01/joint_states 확인{_RESET}"
        l_j1 = l_j2 = l_v1 = l_v2 = l_t1 = l_t2 = ""

    lines = [
        sep,
        f"  {_BOLD}ROBO CHEF{_RESET}  Coord Service "
        f"{_DIM}│{_RESET} {_CYAN}{ROBOT_MODEL}{_RESET} "
        f"{_DIM}│{_RESET} {now}  "
        f"{_DIM}[src=joint_states]{_RESET}",
        f"  서비스: {_DIM}{COORD_SERVICE}{_RESET}   요청: {_CYAN}{req_cnt}회{_RESET}   "
        f"firebase: {_DIM}/{FIREBASE_TELEMETRY}{_RESET}",
        sep, l_top, l_j1, l_j2, l_v1, l_v2, l_t1, l_t2,
    ]
    # 부족한 줄 채우기 (높이 고정)
    while len(lines) < _DISPLAY_HEIGHT:
        lines.append("")
    lines = lines[:_DISPLAY_HEIGHT]

    out = sys.stdout
    if _display_ready:
        out.write(f"\033[{_DISPLAY_HEIGHT}A")
    for line in lines:
        out.write(f"\033[2K{line}\n")
    out.flush()
    _display_ready = True


def _display_loop():
    interval = 1.0 / DISPLAY_HZ
    stale_logged = [False]
    ever_ok      = [False]                     # 최초 OK 도달 전엔 경고 안 찍음
    while True:
        t0 = time.monotonic()
        snap = _snapshot()
        with _lock:
            req_cnt = _cache["req_count"]
        now   = time.monotonic()
        j_gap = (now - _last_joint) if _joint_ready else float("inf")
        ok    = _joint_ready and j_gap <= STALE_SEC

        if ok:
            ever_ok[0] = True
            if stale_logged[0]:
                _log("info", f"[display] 데이터 수신 재개 (joint_gap={j_gap:.2f}s)")
                stale_logged[0] = False
        elif ever_ok[0] and not stale_logged[0]:     # 기동 노이즈 차단, 런타임 단절만 경고
            _log("warn", f"[display] 데이터 미수신/stale — joint({j_gap:.2f}s)")
            stale_logged[0] = True

        try:
            _print_display(snap, ok, req_cnt)
        except Exception as e:
            sys.stderr.write(f"[ERROR][display] {e}\n")
            sys.stderr.flush()
        time.sleep(max(0.0, interval - (time.monotonic() - t0)))


# ════════════════════════════════════════════════════════════════
#  Main
# ════════════════════════════════════════════════════════════════

def main():
    global _node

    rclpy.init()
    node = rclpy.create_node("robo_chef_coord_service")
    _node = node
    _log("info", "[init] 노드 생성: robo_chef_coord_service")

    # ── 콜백 그룹 분리 ──
    #   joint_group   : joint_states 전용 — 별 스레드에서 항상 처리
    #   service_group : 외부 서비스
    #   pub_group     : ROS 토픽 발행
    joint_group   = ReentrantCallbackGroup()
    service_group = MutuallyExclusiveCallbackGroup()
    pub_group     = MutuallyExclusiveCallbackGroup()

    # joint_states — 항상 수신
    node.create_subscription(JointState, TOPIC_JOINTS, _on_joint_states, 10,
                             callback_group=joint_group)
    _log("info", f"[init] 구독: {TOPIC_JOINTS} (전용 콜백 그룹)")

    # 서비스
    node.create_service(Trigger, COORD_SERVICE, _handle_get_coords,
                        callback_group=service_group)
    _log("info", f"[init] 서비스: {COORD_SERVICE}")

    # ROS 토픽
    coord_pub = node.create_publisher(String, COORD_TOPIC, 10)

    def _publish_coords():
        try:
            msg = String()
            msg.data = json.dumps(_snapshot(), ensure_ascii=False)
            coord_pub.publish(msg)
        except Exception as e:
            _log("error", f"[publish] 예외: {e}")

    node.create_timer(1.0 / PUBLISH_HZ, _publish_coords,
                      callback_group=pub_group)
    _log("info", f"[init] 퍼블리셔: {COORD_TOPIC} @ {PUBLISH_HZ}Hz")

    # Firebase 업로더 (별 스레드)
    firebase_pub = _FirebasePublisher()
    firebase_pub.start()

    # SafetyBridge — 전용 CallbackGroup (좌표 스트림과 격리)
    safety_group = ReentrantCallbackGroup()
    safety = SafetyBridge(node, robot_id=ROBOT_ID, callback_group=safety_group)
    safety.start()

    # 배너
    print(f"\n{_BOLD}  ROBO CHEF Coordinate Service{_RESET}")
    print(f"  모델      : {_CYAN}{ROBOT_MODEL}{_RESET}")
    print(f"  입력      : {_CYAN}{TOPIC_JOINTS}{_RESET}")
    print(f"  ROS 서비스: {_CYAN}{COORD_SERVICE}{_RESET}")
    print(f"  ROS 토픽  : {_CYAN}{COORD_TOPIC}{_RESET} @ {PUBLISH_HZ}Hz")
    print(f"  Firebase  : {_CYAN}/{FIREBASE_TELEMETRY}{_RESET} @ {FIREBASE_PUSH_HZ}Hz")
    print(f"  Safety    : listen {_CYAN}/{FB_CMD_PATH}{_RESET} · ack {_CYAN}/{FB_ACK_PATH}{_RESET} · state {_CYAN}/{FB_STATE_PATH}{_RESET}\n")

    # 터미널 디스플레이 스레드
    threading.Thread(target=_display_loop, daemon=True, name="display").start()

    # MultiThreadedExecutor — 콜백 그룹을 독립 스레드에 배치
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        _log("info", "[shutdown] KeyboardInterrupt")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
