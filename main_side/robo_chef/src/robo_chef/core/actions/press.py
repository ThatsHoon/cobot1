from ..base_action import BaseAction

class Press(BaseAction):
    """현재 위치에서 Z축 방향으로 일정한 힘을 주며 누르기"""
    action_name = 'press'

    def execute(self, force_n=15, duration=3.0):
        # 1. Z축 강성을 낮춰서 부드럽게 만듦
        if not self.manager.perform('compliance_on', stx=[3000, 3000, 500, 200, 200, 200], ref='tool'): return False
        
        # 2. 툴 Z축 방향(+ 방향)으로 force_n 만큼의 힘 설정
        if not self.manager.perform('set_desired_force', fd=[0, 0, force_n, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], ref='tool'): return False
        
        # 3. 지정된 시간 동안 꾹 누르고 대기
        self.wait(duration) 
        
        # 4. 해제 및 복구
        if not self.manager.perform('compliance_off'): return False
        return True