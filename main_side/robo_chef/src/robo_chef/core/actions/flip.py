from ..base_action import BaseAction

class Flip(BaseAction):
    action_name = 'flip'
    
    def execute(self, pos):
        if not self.manager.perform('movej', joint=([-60, 0, 0, 0, 0, 0])): return False
        if not self.manager.perform('movej', joint=([0, 0, 0, 0, -90, 0])): return False
        if not self.manager.perform('movel', pos=[0, 0, -350, 0, 0, 0], mode='rel'): return False
        #if not self.manager.perform('movej', joint=([15, 0, 0, 0, 0, 0])): return False
        # if not self.manager.perform('movel', pos=[-300, -500, 0, 0, 0, 0], mode='rel'): return False
        # if not self.manager.perform('movel', pos=[-10, -200, -300, 0, 0, 0], mode='rel'): return False
        
        if not self.manager.perform('movel', pos=pos): return False
        # if not self.manager.perform('movel', pos=([20, 0, 0, 0, 0, 0]), mode='rel'): return False
        if not self.manager.perform('gripper_close'): return False
        if not self.manager.perform('movel', pos=([0, 0, 160, 0, 0, 0]), mode='rel'): return False
        if not self.manager.perform('movej', joint=([0, 0, 0, 0, 0, 180])): return False
        if not self.manager.perform('movel', pos=([0, 0, -160, 0, 0, 0]), mode='rel'): return False
        if not self.manager.perform('gripper_open'): return False
        if not self.manager.perform('movel', pos=([-70, 0, 0, 0, 0, 0]), mode='rel'): return False
        #if not self.manager.perform('movej', joint=([0, 0, 0, 0, 0, -180])): return False
        if not self.manager.perform('movej', joint=([-45, 0, 0, 0, 0, 0])): return False
        return True