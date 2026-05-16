from ..base_action import BaseAction

class Place(BaseAction):
    action_name = 'place'

    def execute(self, pos, height):
        '''place(내려놓는)작업을 위한 위치(pos=list[6]), 상하운동 거리(height=int)'''
        if not self.manager.perform('movel', pos=pos, mode='abs'): return False
        if not self.manager.perform('movel', pos=[0,0,-height,0,0,0], mode='rel'): return False
        if not self.manager.perform('gripper_open'): return False
        if not self.manager.perform('movel', pos=[0,0,height,0,0,0], mode='rel'): return False
        
        return True