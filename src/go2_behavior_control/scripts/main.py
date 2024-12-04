import signal
import sys
import rospy
from std_msgs.msg import String
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from state_machine.FSM import FSM
from common.enums import StateCmd, CommandType
from interface.cmd_handler import Command, CommandHandler
from interface.keyboard import KeyboardInterface


class Go2Manager():
    def __init__(self):
        # 创建命令处理器
        self.command_handler = CommandHandler()
        
        # 启动键盘接口
        self.interface = KeyboardInterface(self.command_handler)
        
        self.voice_sub = rospy.Subscriber('robot_command_topic', String, self.voice_callback)
        
        self.object_label = 'glove'
        self.fsm = FSM()
        self.current_cmd = StateCmd.NONE
        self.running = True      
          
    def voice_callback(self, msg):
        rospy.loginfo('Received voice command: %s', msg.data)
        
        # 语音命令转换为统一的命令格式
        command_map = {
            "站起来": Command(CommandType.ACTION, "stand_up"),
            "趴下": Command(CommandType.ACTION, "stand_down"),
            "前进": Command(CommandType.MOVE, (0.15, 0, 0)),
            # ... 添加更多语音命令映射
        }
        
        if msg.data in command_map:
            self.command_handler.add_command(command_map[msg.data])
    
    def run(self):
        # 查看命令但不移除
        cmd = self.command_handler.peek_command()
        if cmd and cmd.type == CommandType.STATE:
            # 如果是状态切换命令，则处理并移除
            self.fsm.set_next_state(cmd.params)
            self.command_handler.pop_command()
            return
        
        self.fsm.run(self.command_handler)
    
    def stop(self):
        self.running = False
        self.interface.stop()
 
def signal_handler(sig, frame):
    print("停止控制器")
    global go2_manager
    go2_manager.stop()
    
if __name__ == '__main__':
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)
    rospy.init_node('go2_fsm')  
    go2_manager = Go2Manager()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # 打印键盘控制说明
    go2_manager.interface.print_instructions()
    
    try:
        while go2_manager.running:
            go2_manager.run()
            rospy.sleep(0.01)
    except KeyboardInterrupt:
        print("程序已退出")
    finally:
        go2_manager.stop()


