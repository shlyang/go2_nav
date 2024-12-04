from state_machine.BaseState import BaseState
from common.enums import FSMStateName, StateCmd

class FixedstandState(BaseState):
    def __init__(self, ctrl_handler):
        super().__init__(ctrl_handler, FSMStateName.FIXEDSTAND)
        
    def enter(self):
        self._ctrl_handler.stand_up()
    
    def run(self, cmd_handler):
        pass
    
    def exit(self):
        self.state_cmd = StateCmd.NONE
    
    def check_change(self):
        if self.state_cmd == StateCmd.HIGHCMD:
            return FSMStateName.HIGHCMD
        elif self.state_cmd == StateCmd.NAVIGATE:
            return FSMStateName.NAVIGATE
        elif self.state_cmd == StateCmd.PASSIVE:
            return FSMStateName.PASSIVE
        else:
            return FSMStateName.FIXEDSTAND