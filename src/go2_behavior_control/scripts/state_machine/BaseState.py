from abc import ABC, abstractmethod
from common.enums import FSMStateName, StateCmd

class BaseState(ABC):
    def __init__(self, ctrl_handler, state_name):
        self._state_name = state_name
        self._next_state_name = None
        
        self._ctrl_handler = ctrl_handler
        self.state_cmd = StateCmd.NONE

    @abstractmethod
    def enter(self):
        pass

    @abstractmethod
    def run(self, cmd_handler):
        pass

    @abstractmethod
    def exit(self):
        pass

    @abstractmethod
    def check_change(self):
        return FSMStateName.INVALID