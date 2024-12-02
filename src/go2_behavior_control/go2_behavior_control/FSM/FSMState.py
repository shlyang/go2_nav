class FSMState:
    def __init__(self, ctrl_comp, state_name, state_name_string):
        self._ctrl_comp = ctrl_comp
        self._state_name = state_name
        self._state_name_string = state_name_string
        self._next_state_name = None
        self._low_cmd = None
        self._low_state = None
        self._user_value = None

    def enter(self):
        raise NotImplementedError

    def run(self):
        raise NotImplementedError

    def exit(self):
        raise NotImplementedError

    def check_change(self):
        return "INVALID"