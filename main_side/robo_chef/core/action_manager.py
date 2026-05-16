import importlib
import pkgutil
import core.actions as actions

from .base_action import BaseAction

class ActionManager():
    def __init__(self, node):
        self.node = node
        self._action_map = {}
        self.is_error = False   # 시스템 에러 상태 플래그
        
        self._register_methods()
        self._register_custom_actions()
    
    def _register_methods(self):
        """BaseAction의 메서드 등록 (movel, movej, gripper_open 등)"""
        
        # 메서드 가져오기용 인스턴스 생성
        base = BaseAction(self)
        methods = ['movel', 'movej', 'wait', 'reset', 
                   'gripper_open', 'gripper_close', 'gripper_open_little',
                   'compliance_on', 'compliance_off', 'set_desired_force',
                   'periodic', 'clear_alarm']
        
        for m in methods:
            # getattr(obj, name): 객체에서 속성 가져오기
            method = getattr(base, m)
            self._action_map[m] = type(f"Method_{m}", (object,), {"execute": staticmethod(method)})
            self.node.get_logger().info(f"Method Registered: {m}")
            
    def _register_custom_actions(self):
        """actions 폴더 내의 모든 액션들을 자동으로 등록"""
        # 파일 임포트
        for _, name, _ in pkgutil.iter_modules(actions.__path__):
            full_module_name = f"core.actions.{name}"
            importlib.import_module(full_module_name)
        
        # 액션 등록
        for cls in BaseAction.__subclasses__():
            action_name = cls.action_name or cls.__name__.lower()
            
            # 동작 이름에 맞는 인스턴스를 등록
            self._action_map[action_name] = cls(self)
            self.node.get_logger().info(f"Action Registered: {action_name}")

    def perform(self, action_name, **kwargs):
        if self.is_error:
            self.node.get_logger().info("에러 상황으로 perform 중단")
            return False
        
        action = self._action_map.get(action_name)
        if not action:
            self.node.get_logger().info(f"Error: {action_name}을 찾을 수 없습니다.")
            return False

        # callable(action.execute)를 통해 실행 가능 여부만 체크
        execute_func = getattr(action, 'execute', None)
        if callable(execute_func):
            # 파이썬 코드 에러 발생 시에도 안전망이 작동하도록 try-except
            try:
                result = execute_func(**kwargs)
            except Exception as e:
                print(f"🐍 [PYTHON ERROR] {action_name} 파라미터 또는 문법 오류: {e}")
                self.handle_critical_error(action_name) # 에러 시에도 무조건 compliance_off 실행!
                return False
            
            # 동작 실패(False 반환) 감지 시 처리
            if result is False:
                self.handle_critical_error(action_name)
                return False
            
            return True
            
        return False
    
    def handle_critical_error(self, action_name):
        """어떤 동작이든 실패하면 즉시 로봇을 멈추고 예외 모드로 진입"""
        self.is_error = True
        
        # 재시도를 위해 로봇 상태(순응제어, 좌표계) 등 강제 초기화
        try:
            comp_off = self._action_map.get('compliance_off')
            if comp_off:
                comp_off.execute()
        except Exception as e:
            self.node.get_logger().error(f"상태 강제 초기화 중 무시된 오류: {e}")
            
        stop_action = self._action_map.get('stop')
        if stop_action:
            stop_action.execute()
            
        self.node.get_logger().error(f"🚨 [EMERGENCY] {action_name} 수행 중 실패! 시스템을 정지합니다.")