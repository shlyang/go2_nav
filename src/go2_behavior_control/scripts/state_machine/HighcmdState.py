from state_machine.BaseState import BaseState
from common.enums import FSMStateName, StateCmd, CommandType

class HighcmdState(BaseState):
    def __init__(self, ctrl_handler):
        super().__init__(ctrl_handler, FSMStateName.HIGHCMD)
        
    def enter(self):
        self._ctrl_handler.balance_stand()
    
    def run(self, cmd_handler):
        cmd = cmd_handler.pop_command()
        if cmd: 
            if cmd.type == CommandType.ACTION:
                action = cmd.params
                if hasattr(self._ctrl_handler, action):
                        getattr(self._ctrl_handler, action)()
            elif cmd.type == CommandType.MOVE:
                vx, vy, wz = cmd.params
                self._ctrl_handler.move_with_avoid(vx, vy, wz)
    
    def exit(self):
        self.state_cmd = StateCmd.NONE
    
    def check_change(self):
        if self.state_cmd == StateCmd.PASSIVE:
            return FSMStateName.PASSIVE
        elif self.state_cmd == StateCmd.FIXEDSTAND:
            return FSMStateName.FIXEDSTAND
        else:
            return FSMStateName.HIGHCMD
        