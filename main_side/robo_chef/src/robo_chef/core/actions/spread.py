from ..base_action import BaseAction

class Spread(BaseAction):
    """표면에 일정한 압력을 가한 상태로 지그재그나 원형으로 소스를 펴 바르는 동작"""
    action_name = 'spread'

    def execute(self, pos, spread_force=8, amp=[40, 40, 0, 0, 0, 0], period=1.5, repeat=4):
        # 1. 바를 위치의 살짝 위쪽으로 이동
        if not self.manager.perform('movel', pos=pos): return False
        
        # 2. 바닥 밀착을 위해 유연하게 만듦 (특히 Z축을 아주 부드럽게)
        if not self.manager.perform('compliance_on', stx=[2000, 2000, 200, 200, 200, 200], ref='tool'): return False
        
        # 3. Z축(아래) 방향으로 부드러운 힘(예: 8N)을 가해 표면을 꾹 누름
        # 표면의 높낮이가 울퉁불퉁해도 로봇이 힘을 일정하게 유지하며 알아서 오르락내리락 합니다.
        if not self.manager.perform('set_desired_force', fd=[0, 0, spread_force, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], ref='tool'): return False
        self.wait(0.5)
        
        # 4. 표면을 누른 상태에서 periodic을 이용해 X, Y 평면으로 둥글게 펴 바름
        if not self.manager.perform('periodic', amp=amp, period=period, repeat=repeat): return False
        
        # 5. 제어 해제 및 위로 후퇴하여 바르기 종료
        if not self.manager.perform('compliance_off'): return False
        if not self.manager.perform('movel', pos=[0, 0, -100, 0, 0, 0], mode='rel', ref='tool'): return False
        
        return True