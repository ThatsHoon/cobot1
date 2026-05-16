from ..base_action import BaseAction

class Push(BaseAction):
    action_name = 'push'

    def execute(self, pos, height):
        if not self.manager.perform('movel', pos=pos): return False
        if not self.manager.perform('movel', pos=[0, 0, -height, 0, 0, 0], mode='rel'): return False
        if not self.manager.perform('movel', pos=[0, 0, height, 0, 0, 0], mode='rel'): return False
        return True