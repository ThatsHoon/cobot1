from ..base_action import BaseAction

class Pour(BaseAction):
    """용기를 기울여 내용물을 붓고 제자리로 돌아오는 동작"""
    action_name = 'pour'

    def execute(self, pos, angle=100, wait_time=2.0):
        # 1. 냄비 위 안전 고도로 이동
        if not self.manager.perform('approach', pos=pos, offset=150): return False
        
        # 2. 손목(Ry 또는 Rx)을 angle만큼 꺾어서 붓기 (Tool 기준 상대 회전)
        if not self.manager.perform('movel', pos=[0, 0, 0, 0, angle, 0], mode='rel', ref='tool'): return False
        
        # 3. 내용물이 다 떨어질 때까지 대기
        self.wait(wait_time)
        
        # 4. 손목 원상 복구 (반대 방향 회전)
        if not self.manager.perform('movel', pos=[0, 0, 0, 0, -angle, 0], mode='rel', ref='tool'): return False
        
        # 5. 위로 빠져나오기
        if not self.manager.perform('movel', pos=[0, 0, -150, 0, 0, 0], mode='rel', ref='tool'): return False
        
        return True