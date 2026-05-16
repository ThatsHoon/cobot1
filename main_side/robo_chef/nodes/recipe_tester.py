import rclpy
import time
import json
import os

from rclpy.node import Node
from std_msgs.msg import String


class RecipeTester(Node):
    def __init__(self):
        super().__init__('recipe_tester')
        self.publisher = self.create_publisher(String, '/recipe', 10)
        
    def load_recipe_from_file(self, file_name):
        """로컬 data 폴더에서 json 파일을 읽어옵니다."""
        data_path = os.path.join("src/robo_chef/data/", file_name) 
        
        try:
            with open(data_path, 'r', encoding='utf-8') as f:
                recipe_data = json.load(f)
                self.get_logger().info(f"✅ 파일을 성공적으로 불러왔습니다: {data_path}")
                return recipe_data
        except FileNotFoundError:
            self.get_logger().error(f"❌ 파일을 찾을 수 없습니다: {data_path}")
            return None
        except json.JSONDecodeError:
            self.get_logger().error(f"❌ JSON 형식 오류가 발생했습니다: {file_name}")
            return None
        
    def run_test(self, file_name):
        # 로컬 파일에서 데이터 로드
        test_recipe = self.load_recipe_from_file(file_name)
        
        if test_recipe is None:
            return
        
        # Executer가 처리할 수 있도록 JSON 문자열로 변환
        msg = String()
        msg.data = json.dumps(test_recipe, ensure_ascii=False)
        
        # 발행 전 잠시 대기 (퍼블리셔 연결 안정화)
        self.get_logger().info("⏳ 명령 전송 준비 중...")
        time.sleep(1) 
        
        self.publisher.publish(msg)
        self.get_logger().info("테스트 레시피를 전송했습니다.")

def main():
    rclpy.init()
    node = RecipeTester()
    
    test_recipe_name = 'test.json'
    
    try:
        node.run_test(test_recipe_name)
        # 명령 전송 후 노드가 바로 종료되지 않도록 약간 대기
        time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()