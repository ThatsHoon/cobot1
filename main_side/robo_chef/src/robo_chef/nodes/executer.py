import rclpy
import DR_init
import time
import json

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from recipe_msgs.action import Recipe
from core.action_manager import ActionManager

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class RecipeExecuter(Node):
  def __init__(self):
    super().__init__('recipe_executer')
    
    # 초기화
    self.init_dsr()
    self.action_manager = ActionManager(node=self)
    
    self._action_server = ActionServer(
      self,
      Recipe,
      'execute_recipe',
      execute_callback=self.execute_callback,
      goal_callback=self.goal_callback,
      cancel_callback=self.cancel_callback
    )
    self.get_logger().info("🚀 Recipe Action Server Ready.")
    
  def init_dsr(self):
    """두산 로봇의 초기화 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp,get_tool,get_tcp,ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS  # 필요한 기능만 임포트
    from DSR_ROBOT2 import get_robot_mode,set_robot_mode

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)  # 설정 안정화를 위해 잠시 대기
    
    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}") 
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print("#" * 50)
    
  def goal_callback(self, goal_request):
    return GoalResponse.ACCEPT
  
  def cancel_callback(self, goal_handle):
    self.get_logger().warn("🛑 Cancel Request Received!")
    return CancelResponse.ACCEPT
  
  async def execute_callback(self, goal_handle):
    """독립된 스레드에서 실제 로봇 동작 수행"""
    sequence = json.loads(goal_handle.request.recipe_sequence)
    result = Recipe.Result()
    feedback = Recipe.Feedback()
    
    self.action_manager.is_error = False
    self.action_manager.perform('clear_alarm')
    
    for i, step in enumerate(sequence):
      # 중단 요청 확인
      if goal_handle.is_cancel_requested:
        self.action_manager.perform('stop')
        goal_handle.canceled()
        
        result.success = False
        result.message = "Cancelled by user"
        return result
      
      # 피드백 업데이트
      feedback.current_step = i+1
      feedback.current_action = step.get('desc', step['action'])
      goal_handle.publish_feedback(feedback)
      
      self.get_logger().info(f"▶️ Executing: {feedback.current_action}")
      
      # 동작 실행 후 성공여부 확인
      success = self.action_manager.perform(step['action'], **step.get('params', {}))
      if not success:
        try:
          from DSR_ROBOT2 import get_current_posx, DR_BASE
          curr_pos = get_current_posx(DR_BASE)
          pos_str = f" [멈춘 좌표: X:{curr_pos[0]:.1f}, Y:{curr_pos[1]:.1f}, Z:{curr_pos[2]:.1f}]"
        except Exception as e:
          print(f"좌표 캡처 실패: {e}")
          pos_str = ""
          
        # ActionManage 내부에서 stop() 실행 후
        goal_handle.abort()
        result.success = False
        result.message = f"Error at step: {feedback.current_action}{pos_str}"
        return result
    
    goal_handle.succeed()
    result.success = True
    result.message = "Recipe Completed Successfully"
    return result
      

def main(args=None):
    rclpy.init(args=args)
    
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    
    # DSR 내부 통신을 전담할 더미헬퍼 노드
    dsr_node = Node('dsr_helper_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node
    
    try:
      # 전역적으로 DSR_ROBOT2 로드 시도
      import DSR_ROBOT2
    except Exception as e:
      print(f"DSR_ROBOT2 Load Error: {e}")
      
    executor = MultiThreadedExecutor(num_threads=4)
    node = RecipeExecuter()
    
    executor.add_node(dsr_node)
    executor.add_node(node)
    
    try:
      executor.spin()
    except KeyboardInterrupt:
        print("\nKeyboard interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
      node.destroy_node()
      rclpy.shutdown()

if __name__ == "__main__":
    main()