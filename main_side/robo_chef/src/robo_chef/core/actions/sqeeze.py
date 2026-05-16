from ..base_action import BaseAction

class Squeeze(BaseAction):
    """튜브형 소스통을 기울여 소스를 짜내는 동작"""
    action_name = 'squeeze'

    def execute(self, pos, tilt_angle=120, wait_time=2.0):
        # 1. 소스 뿌릴 위치(음식 위)로 이동
        if not self.manager.perform('movel', pos=pos): return False
        
        # 2. 소스통 기울이기 (Tool 기준 Ry 또는 Rx 회전)
        # 병의 입구가 아래를 향하도록 손목을 강하게 꺾어줍니다.
        if not self.manager.perform('movel', pos=[0, 0, 0, 0, tilt_angle, 0], mode='rel', ref='tool'): return False
        
        # 3. 소스가 중력에 의해 충분히 흘러나오도록 대기
        self.wait(wait_time)
        
        # 💡 만약 내용물이 잘 안 나오는 소스라면 여기서 periodic을 짧게 호출해 
        # 위아래로 흔들어주는(Shake) 로직을 결합해도 좋습니다.
        
        # 4. 소스통 다시 똑바로 세우기
        if not self.manager.perform('movel', pos=[0, 0, 0, 0, -tilt_angle, 0], mode='rel', ref='tool'): return False
        
        return True