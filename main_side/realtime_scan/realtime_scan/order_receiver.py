"""
ROBO CHEF — Main Node Order Receiver (Log-only)
================================================
서브노드에서 발행하는 /robo_chef/order_request 토픽을 구독해
수신된 주문·레시피 데이터를 터미널에 출력한다.

실행:
  ros2 run realtime_scan order_receiver
  또는
  python3 realtime_scan/order_receiver.py

수신 메시지 포맷 (JSON):
  {
    "order_id":    "ORD_...",
    "recipe_id":   "RAMEN_001",
    "recipe_name": "라멘",
    "total_steps": 5,
    "locations":   { "INGREDIENTS": {...}, ... },
    "sequence":    [ {"step":1, "action":"HOME", ...}, ... ]
  }
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ORDER_TOPIC = "/robo_chef/order_request"


class OrderLogReceiver(Node):

    def __init__(self):
        super().__init__("robo_chef_order_receiver")
        self._seen: set = set()
        self.create_subscription(String, ORDER_TOPIC, self._on_order, 10)
        self.get_logger().info(f"대기 중 — 구독: {ORDER_TOPIC}")

    def _on_order(self, msg: String):
        try:
            order_id = json.loads(msg.data).get("order_id", "")
        except Exception:
            order_id = ""

        if order_id and order_id in self._seen:
            return
        if order_id:
            self._seen.add(order_id)

        print(msg.data)


def main():
    rclpy.init()
    node = OrderLogReceiver()
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
