from state_machine.BaseState import BaseState
from common.enums import FSMStateName, StateCmd

class PassiveState(BaseState):
    def __init__(self, ctrl_handler):
        super().__init__(ctrl_handler, FSMStateName.PASSIVE)
        
    def enter(self):
        self._ctrl_handle.stand_down()
        self._ctrl_handle.damp()
        
    def run(self, cmd_handler):
        pass
    
    def exit(self):
        self.state_cmd = StateCmd.NONE
    
    def check_change(self):
        if self.state_cmd == StateCmd.FIXEDSTAND:
            # print("change state: PASSIVE -> FIXEDSTAND")
            return FSMStateName.FIXEDSTAND
        else:
            return FSMStateName.PASSIVE