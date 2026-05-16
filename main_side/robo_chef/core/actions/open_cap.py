from ..base_action import BaseAction

class OpenCap(BaseAction):
    action_name = 'open_cap'

    def execute(self, pos):
        if not self.manager.perform('movel', pos=pos): return False
        if not self.manager.perform('compliance_on', stx=[3000, 3000, 500, 200, 200, 200]): return False
        if not self.manager.perform('movel', pos=[0, 0, 0, 0, 0, 90], mode='rel'): return False
        if not self.manager.perform('movel', pos=[0, 0, -150, 0, 0, 0], mode='rel'): return False
        if not self.manager.perform('gripper_close'): return False
        if not self.manager.perform('movel', pos=[0, 0, 0, 0, 0, -90], mode='rel'): return False
        if not self.manager.perform('movel', pos=[0, 0, 150, 0, 0, 0], mode='rel'): return False

        if not self.manager.perform('compliance_off'): return False
        
        return True