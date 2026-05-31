"""
ROBO CHEF — /rosout → Firebase /dsr_log Bridge

rqt_console 이 구독하는 동일 토픽(`/rosout`, rcl_interfaces/msg/Log)을 수집해
Firebase Realtime DB `/dsr_log` 경로에 push. 최대 300개만 유지(오래된 것부터 삭제).

admin.thatshoon.com 은 `/dsr_log` 를 listen 하여 실시간 표시.

주의 (CLAUDE.md dsr-skill)
  - DSR_ROBOT2 import 금지 (g_node 충돌 회피).
  - 자기 자신의 로그는 재발행 방지 (feedback loop 차단).
  - /rosout 은 매우 수다스러움 — DEBUG 는 기본 스킵, 배치 push 로 부하 완화.
"""

import os
import queue
import sys
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import Log


# ── 설정 ─────────────────────────────────────────────────────────
FIREBASE_DB_URL_DEFAULT = "https://robochef-5d9b6-default-rtdb.asia-southeast1.firebasedatabase.app"
FIREBASE_DB_URL    = os.environ.get("FIREBASE_DB_URL", FIREBASE_DB_URL_DEFAULT)
FB_LOG_PATH        = "dsr_log"
MAX_ENTRIES        = 300          # Firebase 에 유지할 최대 개수
# rcl_interfaces/msg/Log level 상수: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
MIN_LEVEL          = 20           # INFO 이상만
BATCH_INTERVAL_SEC = 0.5          # push 배칭 주기
TRIM_EVERY_N_PUSH  = 30           # N건 push 마다 trim 수행
OWN_NODE_NAMES     = {"dsr_log_bridge"}   # 피드백 루프 방지

# FIREBASE_CRED_PATH 환경변수 우선. 없으면 후보 경로 중 첫 번째 존재 파일 사용.
FIREBASE_CRED_CANDIDATES = [
    os.path.expanduser("~/.config/cobot1/firebase-key.json"),
    os.path.expanduser("~/cobot_ws/src/cobot1/main_side/robo_chef/config/serviceAccountKey.json"),
    os.path.expanduser("~/cobot_ws/src/robo_chef/config/serviceAccountKey.json"),
    os.path.expanduser("~/cobot_ws/src/a_robo_chef/config/serviceAccountKey.json"),
    os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "..", "..", "robo_chef", "config",
                                  "serviceAccountKey.json")),
]


def _resolve_firebase_cred() -> str | None:
    env = os.environ.get("FIREBASE_CRED_PATH")
    if env and os.path.isfile(env):
        return env
    return next((p for p in FIREBASE_CRED_CANDIDATES if os.path.isfile(p)), None)

LEVEL_NAMES = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def _lv_to_int(v):
    """msg.level 은 rclpy 버전에 따라 bytes(len 1) 또는 int. 정수로 정규화."""
    if isinstance(v, (bytes, bytearray)):
        return v[0] if len(v) > 0 else 0
    try:
        return int(v)
    except Exception:
        return 0


# ── ANSI (콘솔 표시) ─────────────────────────────────────────────
_C = dict(DIM="\033[2m", BOLD="\033[1m", RESET="\033[0m",
          GREEN="\033[32m", YELLOW="\033[33m", RED="\033[31m", CYAN="\033[36m")


def _stderr(level, msg):
    color = {"INFO": _C["CYAN"], "WARN": _C["YELLOW"],
             "ERROR": _C["RED"], "OK": _C["GREEN"]}.get(level, _C["CYAN"])
    sys.stderr.write(f"{color}{_C['BOLD']}[log_bridge]{_C['RESET']} {msg}\n")
    sys.stderr.flush()


# ════════════════════════════════════════════════════════════════
class LogBridgeNode(Node):
    def __init__(self):
        super().__init__("dsr_log_bridge")
        self._q: "queue.Queue[dict]" = queue.Queue()

        # /rosout 전용 QoS — ROS2 Humble 의 rcl_logging 기본 프로필과 맞춤
        # (기본 QoS 로 구독하면 incompatible 경고 + 메시지 수신 실패)
        rosout_qos = QoSProfile(
            depth=1000,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Log, "/rosout", self._on_log, rosout_qos)
        self.get_logger().info(f"구독 시작: /rosout → Firebase /{FB_LOG_PATH}")

        self._fb_ref = None
        self._init_firebase()

        if self._fb_ref is not None:
            threading.Thread(target=self._push_loop, daemon=True,
                             name="log-push").start()

    # ── /rosout 수신 콜백 ────────────────────────────────────────
    def _on_log(self, msg: Log):
        level_int = _lv_to_int(msg.level)
        # DEBUG 스킵
        if level_int < MIN_LEVEL:
            return
        # 자기 자신은 스킵 (피드백 방지)
        if msg.name in OWN_NODE_NAMES:
            return

        sec  = int(msg.stamp.sec)
        nsec = int(msg.stamp.nanosec)
        ts = datetime.fromtimestamp(sec + nsec / 1e9, tz=timezone.utc).isoformat()

        entry = {
            "timestamp": ts,
            "level":     LEVEL_NAMES.get(level_int, str(level_int)),
            "node":      msg.name or "?",
            "message":   msg.msg or "",
            "file":      msg.file or "",
            "function":  msg.function or "",
            "line":      int(msg.line) if msg.line else 0,
            "source":    "rosout",
        }
        try:
            self._q.put_nowait(entry)
        except queue.Full:
            pass

    # ── Firebase 초기화 ──────────────────────────────────────────
    def _init_firebase(self):
        try:
            import firebase_admin
            from firebase_admin import credentials, db
        except ImportError:
            _stderr("WARN", "firebase_admin 미설치 — 비활성화")
            return
        cred_path = _resolve_firebase_cred()
        if cred_path is None:
            _stderr("WARN",
                    "credential 파일 미탐지 — FIREBASE_CRED_PATH 환경변수 또는 "
                    f"후보 경로 필요: {FIREBASE_CRED_CANDIDATES}")
            return
        try:
            if not firebase_admin._apps:
                cred = credentials.Certificate(cred_path)
                firebase_admin.initialize_app(cred, {"databaseURL": FIREBASE_DB_URL})
            self._fb_ref = db.reference(FB_LOG_PATH)
            _stderr("OK", f"Firebase 연결 완료 — /{FB_LOG_PATH} (max={MAX_ENTRIES})")
        except Exception as e:
            _stderr("ERROR", f"Firebase 초기화 실패: {e}")
            self._fb_ref = None

    # ── Push 루프 ────────────────────────────────────────────────
    def _push_loop(self):
        push_count = 0
        err_streak = 0
        while rclpy.ok():
            batch = []
            t_end = time.monotonic() + BATCH_INTERVAL_SEC
            while time.monotonic() < t_end and len(batch) < 50:
                try:
                    item = self._q.get(timeout=max(0.01, t_end - time.monotonic()))
                    batch.append(item)
                except queue.Empty:
                    break
            if not batch:
                continue

            for entry in batch:
                try:
                    self._fb_ref.push(entry)
                    push_count += 1
                    err_streak = 0
                except Exception as e:
                    err_streak += 1
                    if err_streak in (1, 10, 100):
                        _stderr("ERROR", f"push 실패(x{err_streak}): {e}")

            if push_count >= TRIM_EVERY_N_PUSH:
                push_count = 0
                self._trim()

    # ── 오래된 엔트리 trim ──────────────────────────────────────
    def _trim(self):
        try:
            # shallow=True 로 키만 받아 전체 size 최소화
            data = self._fb_ref.get(shallow=True)
            if not isinstance(data, dict):
                return
            keys = sorted(data.keys())   # push key 는 시간순 정렬
            excess = len(keys) - MAX_ENTRIES
            if excess <= 0:
                return
            for k in keys[:excess]:
                try:
                    self._fb_ref.child(k).delete()
                except Exception:
                    pass
        except Exception as e:
            _stderr("WARN", f"trim 실패: {e}")


# ════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = LogBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
