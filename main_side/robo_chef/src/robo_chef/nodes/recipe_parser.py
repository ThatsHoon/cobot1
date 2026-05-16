import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class RecipeParser(Node):
    def __init__(self):
        super().__init__('recipe_orchestrator')
        
        self.recipe_sub = self.create_subscription(String, '/recipe', self.recipe_callback, 10)
        self.cmd_pub = self.create_publisher(String, '/parsed_recipe', 10)

    def recipe_callback(self, msg):
        try:
            raw_data = json.loads(msg.data)
            
            # 전처리: 만약 데이터가 메뉴명(예: 'RAMEN')으로 감싸져 있다면 내부 데이터만 추출
            # 'locations'가 키값에 없다면, 가장 첫 번째 키의 밸류를 실제 레시피 데이터로 취급함
            if 'locations' not in raw_data and len(raw_data) > 0:
                first_key = list(raw_data.keys())[0]
                self.get_logger().info(f"📦 '{first_key}' 메뉴의 레시피 데이터를 추출합니다.")
                raw_data = raw_data[first_key]
                
            executable_steps = self.parse_recipe(raw_data)
            
            if executable_steps:
                cmd_msg = String()
                cmd_msg.data = json.dumps(executable_steps, ensure_ascii=False) 
                self.cmd_pub.publish(cmd_msg)
                self.get_logger().info(f"🚀 {len(executable_steps)} 단계의 실행 명령을 전송했습니다.")
                
        except Exception as e:
            self.get_logger().error(f"❌ 레시피 파싱 중 에러 발생: {e}")

    def parse_recipe(self, recipe_data):
        """규격화된 JSON을 바탕으로 한 패스스루 방식의 파싱"""
        
        locations = recipe_data.get('locations', {})
        parsed_sequence = []
        
        # Sequence 순회 (enumerate를 사용해 자동으로 step 부여)
        for i, item in enumerate(recipe_data.get('sequence', [])):
            action = item['action'].lower()
            params = item.get('params', {}).copy()

            # 위치 이름(String)을 실제 좌표(List)로 치환
            if 'pos' in params and isinstance(params['pos'], str):
                target_name = params['pos']
                if target_name in locations:
                    params['pos'] = locations[target_name]
                else:
                    self.get_logger().error(f"⚠️ '{target_name}' 좌표 정보가 없습니다.")

            # 정제된 시퀀스 구성 (데이터 내의 step 대신 enumerate의 i 사용 가능)
            parsed_sequence.append({
                "step": i + 1,  # 로깅이나 디버깅용으로 파서가 자동으로 부여
                "action": action,
                "params": params,
                "desc": item.get('desc', '')
            })
        
        return parsed_sequence
    
def main(args=None):
    rclpy.init(args=args)
    node = RecipeParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()