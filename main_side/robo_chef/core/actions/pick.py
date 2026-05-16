from ..base_action import BaseAction

class Pick(BaseAction):
    action_name = 'pick'

    def execute(self, pos, height):
        if not self.manager.perform('movel', pos=pos, mode='abs'): return False
        if not self.manager.perform('movel', pos=[0, 0, -height, 0, 0, 0], mode='rel'): return False
        if not self.manager.perform('gripper_close'): return False
        if not self.manager.perform('movel', pos=[0, 0, height, 0, 0, 0], mode='rel'): return False
        
        return True