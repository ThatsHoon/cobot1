from ..base_action import BaseAction

class Stir(BaseAction):
    """냄비 바닥에 밀착한 상태로 부드럽게 원을 그리며 젓기"""
    action_name = 'stir'

    def execute(self, pos, duration=5.0):
        # 1. 냄비 내부로 이동 (바닥 근처까지)
        if not self.manager.perform('approach', pos=pos, offset=20): return False
        
        # 2. 바닥 밀착을 위한 컴플라이언스 및 약한 힘(5N) 제어 켬
        if not self.manager.perform('compliance_on', stx=[2000, 2000, 300, 200, 200, 200], ref='tool'): return False
        if not self.manager.perform('set_desired_force', fd=[0, 0, 5, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], ref='tool'): return False
        self.wait(0.5)

        # 3. 주기적 운동 (원형 젓기)
        # X, Y축으로 진폭 40mm, 주기 2초로 빙글빙글 돕니다.
        repeat_count = max(1, int(duration / 2.0))
        if not self.manager.perform('periodic', amp=[40, 40, 0, 0, 0, 0], period=2.0, repeat=repeat_count): return False
        
        # 4. 제어 해제 및 안전하게 위로 뽑아내기
        if not self.manager.perform('compliance_off'): return False
        if not self.manager.perform('movel', pos=[0, 0, -200, 0, 0, 0], mode='rel', ref='tool'): return False
        
        return True