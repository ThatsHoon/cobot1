"""firebase_bridge — RTDB /orders(pending) 감지 → /recipe 잡 발행,
/cooking_status → robot_status + /orders status 전이.

order_count 방식 폐기. 동시 1건(busy), order_time FIFO.
"""
import os
import json
import datetime
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import firebase_admin
from firebase_admin import credentials, db

import order_core as oc

# Firebase 자격증명 해석 순서: FIREBASE_CRED_PATH 환경변수 → 후보 리스트.
# 워크스페이스 위치/사용자 디렉토리가 PC 마다 다르므로 하드코딩을 피한다.
DB_URL_DEFAULT = "https://robochef-5d9b6-default-rtdb.asia-southeast1.firebasedatabase.app"
CRED_CANDIDATES = [
    os.path.expanduser("~/.config/cobot1/firebase-key.json"),
    os.path.expanduser("~/cobot_ws/src/cobot1/main_side/robo_chef/config/serviceAccountKey.json"),
    os.path.expanduser("~/cobot_ws/src/robo_chef/config/serviceAccountKey.json"),
]


def _resolve_cred_path() -> str | None:
    env = os.environ.get("FIREBASE_CRED_PATH")
    if env and os.path.exists(env):
        return env
    for p in CRED_CANDIDATES:
        if os.path.exists(p):
            return p
    return None


def _resolve_db_url() -> str:
    return os.environ.get("FIREBASE_DB_URL", DB_URL_DEFAULT)


class FirebaseBridge(Node):
    def __init__(self):
        super().__init__("firebase_bridge")
        self.recipe_pub = self.create_publisher(String, "/recipe", 10)
        self.create_subscription(String, "/cooking_status",
                                 self._on_status, 10)
        self._busy = False
        self._cur_order = None
        self._lock = threading.Lock()
        cred_path = _resolve_cred_path()
        db_url = _resolve_db_url()
        if not cred_path:
            self.get_logger().error(
                "❌ Firebase 자격증명 없음 — FIREBASE_CRED_PATH 환경변수 또는 "
                f"후보 경로 중 하나 필요: {CRED_CANDIDATES}")
        else:
            try:
                cred = credentials.Certificate(cred_path)
                firebase_admin.initialize_app(cred, {"databaseURL": db_url})
                self.get_logger().info(
                    f"✅ Firebase Connected (cred={cred_path})")
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"❌ Firebase init 실패: {e}")
        self.orders_ref = db.reference("orders")
        self.status_ref = db.reference("robot_status")
        self.log_ref = db.reference("error_logs")
        # 노드 재기동 직후, 이전 인스턴스가 cooking 중 죽어 남긴 orphan 정리.
        # in-memory _cur_order 가 비어있으면 어떤 cooking 주문도 우리 것이 아님 →
        # status=failed 로 명시 종결해 RTDB 가 영구 "cooking" 으로 잠기는 일을 막는다.
        self._recover_orphans()
        self._try_dispatch_next()                      # 기동 시 1회 스캔
        self.orders_ref.listen(self._on_orders_event)  # 이후 실시간

    def _recover_orphans(self):
        try:
            orders = self.orders_ref.get() or {}
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"orphan recovery 스킵 (fetch 실패): {e}")
            return
        recovered = 0
        for oid, o in orders.items():
            if not isinstance(o, dict):
                continue
            if o.get("status") == "cooking":
                db.reference(f"orders/{oid}").update({
                    "status": "failed",
                    "failed_reason": "bridge restart — orphaned cooking order",
                    "completed_at": _now(),
                })
                recovered += 1
        if recovered:
            self.get_logger().warn(
                f"⚠ orphan cooking 주문 {recovered} 건 → failed 로 정리 (재기동 복구)")

    # ---- 주문 인지/디스패치 ----
    def _on_orders_event(self, event):
        self._try_dispatch_next()

    def _try_dispatch_next(self):
        with self._lock:
            if self._busy:
                return
            orders = self.orders_ref.get() or {}
            oid, order = oc.select_next_pending(orders)
            if not oid:
                return
            try:
                jobs = oc.build_jobs(order)
            except oc.OrderError as e:
                self.get_logger().error(f"주문 {oid} 전개 실패: {e}")
                db.reference(f"orders/{oid}").update(
                    {"status": "failed", "error_msg": str(e)})
                return
            self._busy = True
            self._cur_order = oid
            db.reference(f"orders/{oid}").update(
                {"status": "cooking", "started_at": _now()})
            msg = String()
            msg.data = json.dumps({"order_id": oid, "jobs": jobs})
            try:
                self.recipe_pub.publish(msg)
            except Exception as e:  # noqa: BLE001
                # publish 실패 = 디스패치 미완료. RTDB 의 cooking 표기를 되돌리고
                # busy 해제해서 다음 시도가 같은 주문(또는 다음 주문)을 픽업하게 한다.
                self.get_logger().error(
                    f"❌ /recipe publish 실패 → 주문 {oid} pending 으로 복귀: {e}")
                db.reference(f"orders/{oid}").update(
                    {"status": "pending", "started_at": None})
                self._busy = False
                self._cur_order = None
                return
            self.get_logger().info(f"▶️ 주문 {oid} 디스패치 ({len(jobs)} jobs)")

    # ---- 상태 수신 ----
    def _on_status(self, msg: String):
        try:
            st = json.loads(msg.data)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"잘못된 /cooking_status: {e}")
            return
        # 패널 (customer_status, admin_monitor) 가 stale 검출에 사용.
        # cooking_core 가 emit 시점에 채우지 않고 RTDB 직전 단계에서 부여 — 네트워크
        # 시각이 아니라 메시지 발행 시각을 단일 출처로 유지.
        st["last_updated"] = _now()
        self.status_ref.set(st)
        new_status, release = oc.order_transition(st.get("state", ""))
        oid = st.get("order_id") or self._cur_order
        if new_status and oid:
            extra = {"delivered_at": _now()} if new_status == "delivered" \
                else {"error_msg": st.get("error_msg", "")}
            db.reference(f"orders/{oid}").update(
                {"status": new_status, **extra})
            if new_status == "failed":
                self.log_ref.push({
                    "timestamp": _now(), "order_id": oid,
                    "recipe_id": st.get("recipe_id", ""),
                    "item_index": st.get("item_index", 0),
                    "segment_name": st.get("segment_name", ""),
                    "message": st.get("error_msg", "")})
        if release:                       # DONE 또는 IDLE(unlock 재개)
            with self._lock:
                self._busy = False
                self._cur_order = None
            self._try_dispatch_next()


def _now():
    return datetime.datetime.utcnow().isoformat() + "Z"


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
