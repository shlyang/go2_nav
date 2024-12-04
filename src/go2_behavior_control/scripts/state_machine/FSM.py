from state_machine.PassiveState import PassiveState
from state_machine.FixedstandState import FixedstandState
from state_machine.HighcmdState import HighcmdState
from state_machine.NavigateState import NavigateState

from common.enums import FSMMode, FSMStateName

from common.atom_action import AtomAction

class FSM:
    def __init__(self):
        self.atom_action = AtomAction()
        self._stateList = {
            FSMStateName.INVALID: None,
            FSMStateName.PASSIVE: PassiveState(self.atom_action),
            FSMStateName.FIXEDSTAND: FixedstandState(self.atom_action),
            FSMStateName.HIGHCMD: HighcmdState(self.atom_action),
            FSMStateName.NAVIGATE: NavigateState(self.atom_action),
        }
        
        self._current_state = self._stateList[FSMStateName.FIXEDSTAND]
        self._mode = FSMMode.NORMAL
        self._current_state.enter()
        self._next_state = self._current_state

    def run(self, cmd_handler):
        if self._mode == FSMMode.NORMAL:
            self._current_state.run(cmd_handler)
            self._next_state_name = self._current_state.check_change()
            if self._next_state_name != self._current_state._state_name:
                # print(self._next_state_name)
                # print(self._current_state._state_name)
                self._mode = FSMMode.CHANGE
                self._next_state = self.get_next_state(self._next_state_name)
                # print(self._next_state)
                print(f"change state: " + 
                      self._current_state._state_name.value + " -> " +
                      self._next_state._state_name.value)
                
        elif self._mode == FSMMode.CHANGE:
            self._current_state.exit()
            self._current_state = self._next_state
            self._current_state.enter()
            self._mode = FSMMode.NORMAL
            # self._current_state.run()
            
    def get_next_state(self, state_name):
        return self._stateList.get(state_name, self._stateList[FSMStateName.INVALID])
        
    def set_next_state(self, state_name):
        self._current_state.state_cmd = state_name
        