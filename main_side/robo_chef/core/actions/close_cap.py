from ..base_action import BaseAction

class CloseCap(BaseAction):
    action_name = 'close_cap'

    def execute(self, pos, height=150, angle=90): 
        if not self.manager.perform('movel', pos=pos): return False
        if not self.manager.perform('compliance_on', stx=[3000, 3000, 500, 200, 200, 200]): return False
        if not self.manager.perform('movel', pos=[0, 0, -height, 0, 0, 0], mode='rel'): return False
        if not self.manager.perform('gripper_close'): return False
        
        # 내려간 상태에서 movel을 이용해 손목만 90도 회전 (movej 대신 사용!)
        if not self.manager.perform('movel', pos=[0, 0, 0, 0, 0, angle], mode='rel'): return False
        
        if not self.manager.perform('gripper_open_little'): return False
        if not self.manager.perform('movel', pos=[0, 0, height, 0, 0, 0], mode='rel'): return False
        
        # 순응 제어 OFF
        if not self.manager.perform('compliance_off'): return False
        if not self.manager.perform('gripper_open'): return False
        return True

