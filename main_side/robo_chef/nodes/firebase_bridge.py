import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db
from std_msgs.msg import String
import json
import datetime

class FirebaseBridgeNode(Node):
    def __init__(self):
        super().__init__('firebase_bridge')
        
        self.recipe_pub = self.create_publisher(String, '/recipe', 10)
        self.status_pub = self.create_subscription(String, '/cooking_status', self._on_status_receive, 10)
        
        # Firebase 초기화
        try:
            cred = credentials.Certificate('/home/kibeom/cobot_ws/src/robo_chef/config/serviceAccountKey.json')
            firebase_admin.initialize_app(cred, {
                'databaseURL': "https://robochef-5d9b6-default-rtdb.asia-southeast1.firebasedatabase.app"
            })
            self.get_logger().info('✅ Firebase Connected!')
        except Exception as e:
            self.get_logger().error(f'❌ Firebase Connection Failed: {e}')

        # DB 리스너 시작 (실시간 감시)
        self.recipe_ref = db.reference('recipes') # 주문이 들어오는 경로
        self.status_ref = db.reference('robot_status')
        self.log_ref = db.reference('error_logs')
        
        self.recipe_ref.listen(self._on_recipe_change)

    def _on_recipe_change(self, event):
      # event.path 예시: "/RAMEN/order_count"
        path_parts = event.path.strip('/').split('/')
        
        if not path_parts or len(path_parts) < 2:
            return

        # 첫 번째 파트가 요리 이름이 됩니다.
        target_recipe = path_parts[0] # "RAMEN" 또는 "KIMCHI"
        
        if 'order_count' in event.path:
            self.get_logger().info(f"🔔 {target_recipe} 주문 감지!")
            
            # 전체가 아닌, 바뀐 그 요리의 데이터만 가져옵니다.
            specific_recipe_data = self.recipe_ref.child(target_recipe).get()
            
            # 토픽 발행
            msg = String()
            msg.data = json.dumps({target_recipe: specific_recipe_data})
            
            print(msg)
            self.recipe_pub.publish(msg)
            
    def _on_status_receive(self, msg):
        """요리 상태를 DB에 업데이트"""
        try:
            status_data = json.loads(msg.data)
            
            # 진행 상황 UI용
            self.status_ref.set(status_data)
            
            # 에러가 있을 경우 로그 전송
            if status_data.get("state") in ["ERROR", "ADMIN_INTERVENTION"] and status_data.get("error_msg"):
                now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                self.log_ref.push({
                    "timestamp": now,
                    "step": status_data.get("current_step"),
                    "action": status_data.get("current_action"),
                    "message": status_data.get("error_msg")
                })
                
        except Exception as e:
            self.get_logger().error(f'❌ 상태 DB 업데이트 실패: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FirebaseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()