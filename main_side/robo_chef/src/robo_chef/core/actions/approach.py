from ..base_action import BaseAction

class Approach(BaseAction):
    """목표 자세와 지점 위로 안전하게 이동한 뒤, 지정된 거리만큼 수직 하강"""
    action_name = 'approach'

    def execute(self, pos, offset=100):
        # offset만큼 Z축이 높은 안전 위치로 절대 이동
        safe_pos = pos.copy()
        safe_pos[2] += offset 
        if not self.manager.perform('movel', pos=safe_pos, mode='abs', ref='base'): return False
        
        # 목표 지점(아래)으로 조심스럽게 수직 하강 (Tool 기준 전진)
        if not self.manager.perform('movel', pos=[0, 0, offset, 0, 0, 0], mode='rel', ref='tool'): return False
        return True