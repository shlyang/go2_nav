from enum import Enum

class FSMStateName(Enum):
    INVALID = "INVALID"
    PASSIVE = "PASSIVE"
    FIXEDSTAND = "FIXEDSTAND"
    HIGHCMD = "HIGHCMD"
    NAVIGATE = "NAVIGATE"
    
class FSMMode(Enum):
    NORMAL = "NORMAL"
    CHANGE = "CHANGE"
    
class StateCmd(Enum):
    NONE = None
    PASSIVE = "PASSIVE"
    FIXEDSTAND = "FIXEDSTAND"
    HIGHCMD = "HIGHCMD"
    NAVIGATE = "NAVIGATE"
    
class CommandType(Enum):
    MOVE = "MOVE"           # 移动指令
    ACTION = "ACTION"       # 动作指令
    STATE = "STATE"         # 状态切换指令


if __name__ == "__main__":
    print("test enum class")
    cmd = CommandType.MOVE
    print(cmd.value)
    print(type(cmd.value))
