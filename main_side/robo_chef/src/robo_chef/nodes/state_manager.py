import rclpy
import json

from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

from recipe_msgs.action import Recipe

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        
        # 로봇 상태 관리
        self.state = "IDLE"
        self.current_sequence = []
        
        # 에러처리 관련 변수
        self.current_step_index = 0
        self.retry_count = 0
        self.max_retries = 2
        self.recovery_timer = None
        self.sliced_offset = 0
        
        # 파싱된 레시피 받는 토픽, 실행자와 액션 클라이언트, 관리재 잠금 해제 서비스
        self.recipe_sub = self.create_subscription(String, '/parsed_recipe', self.recipe_callback, 10)
        self._action_client = ActionClient(self, Recipe, 'execute_recipe')
        self.unlock_srv = self.create_service(Trigger, 'unlock_system', self.unlock_callback)
        self.status_pub = self.create_publisher(String, '/cooking_status', 10)
        
        self.get_logger().info("🧠 State Manager가 대기 중입니다 (상태: IDLE).")
        
    def recipe_callback(self, msg):
        """파서로부터 레시피 받았을 때의 처리"""
        if self.state != "IDLE":
            self.get_logger().warn(f"⚠️ 현재 로봇이 '{self.state}' 상태입니다. 새 레시피를 무시합니다.")
            return
        
        try:
            self.current_sequence = json.loads(msg.data)
            self.get_logger().info(f"📥 {len(self.current_sequence)}단계의 레시피 수신. 실행을 준비합니다.")
            
            # 새 레시피 수신 시 초기화
            self.retry_count = 0
            self.current_step_index = 0
            self.sliced_offset = 0
            
            self.send_goal_to_executer(self.current_sequence)
        except Exception as e:
            self.get_logger().error(f"❌ 데이터 로드 실패: {e}")
            
    def send_goal_to_executer(self, sequence, is_recovery=False):
        """Executer로 Action Goal 전송 (정상 실행과 재시도/복구 모두 사용)"""
        if not is_recovery:
            self.state = "EXECUTING"
            
        goal_msg = Recipe.Goal()
        goal_msg.recipe_sequence = json.dumps(sequence, ensure_ascii=False)
        
        self.get_logger().info("⏳ Executer 연결 대기 중...")
        self._action_client.wait_for_server()
        
        if is_recovery:
            self.get_logger().info("🚑 에러 복구를 위한 동작 명령 하달!")
        else:
            self.get_logger().info("🚀 요리 동작 명령 하달!")
            
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Executer가 실행을 거부했습니다.")
            self.state = "IDLE"
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        
        # step 인덱스 추적
        self.current_step_index = self.sliced_offset + (feedback.current_step - 1)
        actual_step = self.current_step_index + 1                
        
        # UI에 표시할 이름 저장 후 상태전송
        self.last_action_name = feedback.current_action
        self.publish_status()
        
        self.get_logger().info(f"🔄 [진행중: {actual_step}/{len(self.current_sequence)}] {feedback.current_action}")
        
    def get_result_callback(self, future):
        result = future.result().result
        
        if result.success:
            if self.state == "RECOVERING_RESET":
                self.get_logger().info("✅ 에러 복구(원점 회귀) 성공. 관리자 점검 후 다시 시작해 주세요.")
            else:
                self.get_logger().info(f"🎉 요리 완료: {result.message}")
            
            self.state = "IDLE"
            self.retry_count = 0
            self.last_action_name = "요리 완료"
            self.publish_status()
        else:
            self.get_logger().error(f"🚨 로봇 동작 실패 보고 수신: {result.message}")
            self.last_error_message = result.message
            self.handle_error()
            
    def handle_error(self):
        """예외 상황 복구 시나리오"""
        
        if self.state == "RECOVERING_RESET":
            # 원점 회귀조차 실패하면 최후 상태
            self.state = "RECOVERING_FAIL"
            self.get_logger().fatal("💀 치명적 오류: 원점 복귀조차 실패했습니다! 관리자 개입이 필수적입니다.")
            return
        
        error_text = getattr(self, 'last_error_message', '알 수 없는 동작 에러')
        
        # 일반 재시도 로직
        if self.retry_count < self.max_retries:
            self.state = "RECOVERING_RETRY"
            self.retry_count += 1
            
            # DB로 현재 상태 전송
            self.publish_status(error_msg=f"{error_text} -> 5초 후 재시도합니다. ({self.retry_count}/{self.max_retries})")
            
            self.get_logger().warn(f"🛠️ 5초 대기 후 실패한 단계부터 재시도를 시작합니다. (재시도 횟수: {self.retry_count}/{self.max_retries})")
            
            # 5초 뒤에 retry_execution을 비동기적으로 실행
            self.recovery_timer = self.create_timer(5.0, self.retry_execution)
            
        # 최대 재시도 초과 시 원점 복귀 시도
        else:
            self.get_logger().error("🚨 최대 재시도 횟수(2회) 초과! 원점(Home) 복귀를 시도합니다.")
            self.try_reset()
    
    def retry_execution(self):
        """5초 대기 후 호출되어 남은 시퀸스"""
        if self.recovery_timer:
            self.recovery_timer.cancel()    # 1회용 타이머 정지
        
        self.get_logger().info(f"🔄 중단된 지점(Step {self.current_step_index + 1})부터 재시작합니다.")
        
        self.sliced_offset = self.current_step_index
        remaining_sequence = self.current_sequence[self.current_step_index:]
        self.send_goal_to_executer(remaining_sequence)
        
    def try_reset(self):
        """복구용 원점 회귀 시퀀스 전송"""
        reset_sequence = [{
            "action": "reset", 
            "params": {}, 
            "desc": "에러 복구 원점 회귀"
            }]
        
        self.send_goal_to_executer(reset_sequence, is_recovery=True)
        
        self.state = "RECOVERING_RESET"
        self.publish_status(error_msg="최대 재시도 초과. 안전을 위해 원점 복구를 진행합니다.")
        self.send_goal_to_executer(reset_sequence, is_recovery=True)
    
    def unlock_callback(self, request, response):
        """관리자 개입 후 시스템 잠금을 해제하는 콜백"""   
        if self.state in ["ADMIN_INTERVENTION", "ERROR", "RECOVERING_FAIL"]:
            self.state = "IDLE"
            self.retry_count = 0
            
            self.get_logger().info("🔓 [ADMIN] 관리자에 의해 시스템 잠금이 해제되었습니다. 다시 명령을 받을 수 있습니다.")
            
            # 잠금 해제 시 에러 메시지 초기화 후 상태 퍼블리시
            self.publish_status(error_msg="")
            
            response.success = True
            response.message = "System unlocked successfully."
        else:
            self.get_logger().warn(f"⚠️ 현재 시스템이 잠겨있지 않습니다. (현재 상태: {self.state})")
            response.success = False
            response.message = "System is not in a locked state."
            
        return response
    
    def publish_status(self, error_msg=""):
        """현재 로봇 상태 JSON으로 묶어 퍼블리싱"""
        status_data = {
            "state": self.state,  # IDLE, EXECUTING, ERROR, RECOVERING 등
            "current_step": self.current_step_index + 1 if self.current_sequence else 0,
            "total_steps": len(self.current_sequence),
            "current_action": getattr(self, 'last_action_name', "대기 중"),
            "error_msg": error_msg
        }
        
        msg = String()
        msg.data = json.dumps(status_data, ensure_ascii=False)
        self.status_pub.publish(msg)
        
    
    
def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()