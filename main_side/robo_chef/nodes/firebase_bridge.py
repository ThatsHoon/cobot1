"""firebase_bridge — RTDB /orders(pending) 감지 → /recipe 잡 발행,
/cooking_status → robot_status + /orders status 전이.

order_count 방식 폐기. 동시 1건(busy), order_time FIFO.
"""
import json
import datetime
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import firebase_admin
from firebase_admin import credentials, db

import order_core as oc

CRED_PATH = "/home/kibeom/cobot_ws/src/robo_chef/config/serviceAccountKey.json"
DB_URL = "https://robochef-5d9b6-default-rtdb.asia-southeast1.firebasedatabase.app"


class FirebaseBridge(Node):
    def __init__(self):
        super().__init__("firebase_bridge")
        self.recipe_pub = self.create_publisher(String, "/recipe", 10)
        self.create_subscription(String, "/cooking_status",
                                 self._on_status, 10)
        self._busy = False
        self._cur_order = None
        self._lock = threading.Lock()
        try:
            cred = credentials.Certificate(CRED_PATH)
            firebase_admin.initialize_app(cred, {"databaseURL": DB_URL})
            self.get_logger().info("✅ Firebase Connected")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"❌ Firebase init 실패: {e}")
        self.orders_ref = db.reference("orders")
        self.status_ref = db.reference("robot_status")
        self.log_ref = db.reference("error_logs")
        self._try_dispatch_next()                      # 기동 시 1회 스캔
        self.orders_ref.listen(self._on_orders_event)  # 이후 실시간

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
            self.recipe_pub.publish(msg)
            self.get_logger().info(f"▶️ 주문 {oid} 디스패치 ({len(jobs)} jobs)")

    # ---- 상태 수신 ----
    def _on_status(self, msg: String):
        try:
            st = json.loads(msg.data)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"잘못된 /cooking_status: {e}")
            return
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
