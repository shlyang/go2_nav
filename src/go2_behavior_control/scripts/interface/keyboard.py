import sys
import select
import termios
import tty
import threading
from interface.cmd_handler import Command
from common.enums import StateCmd, CommandType
import rospy
import yaml
import os
import rospkg


class KeyboardInterface:
    def __init__(self, command_handler):
        self.command_handler = command_handler
        self.running = True
        
        # 保存终端设置
        self.old_settings = termios.tcgetattr(sys.stdin)
        # 设置新的终端属性
        new_settings = termios.tcgetattr(sys.stdin)
        new_settings[3] = new_settings[3] & ~(termios.ECHO | termios.ICANON)  # 关闭回显和规范模式
        termios.tcsetattr(sys.stdin, termios.TCSANOW, new_settings)
        
        # 从YAML文件加载按键映射
        self._load_key_mappings()
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
    def _load_key_mappings(self):
        """从YAML文件加载按键映射配置"""
        try:
            # 获取包路径
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('go2_fsm')
            yaml_path = os.path.join(pkg_path, 'config', 'keyboard_map.yaml')
            
            with open(yaml_path, 'r') as f:
                key_map = yaml.safe_load(f)
            
            # 将状态命令字符串转换为枚举
            self.state_map = {k: getattr(StateCmd, v) for k, v in key_map['state_commands'].items()}
            self.movement_map = key_map['movement_commands']
            self.action_map = key_map['action_commands']
            
        except Exception as e:
            rospy.logerr(f"加载按键映射配置失败: {e}")
            # 使用默认映射作为备份
            self._load_default_mappings()
            
    def _load_default_mappings(self):
        """加载默认按键映射"""
        self.state_map = {
            '1': StateCmd.PASSIVE,
            '2': StateCmd.FIXEDSTAND,
            '3': StateCmd.HIGHCMD,
            '4': StateCmd.NAVIGATE
        }
        
        self.movement_map = {
            'w': (0.15, 0, 0),     # 前进
            's': (-0.15, 0, 0),    # 后退
            'a': (0, 0.15, 0),     # 左移
            'd': (0, -0.15, 0),    # 右移
            'q': (0, 0, 0.5),      # 左转
            'e': (0, 0, -0.5),     # 右转
        }
        
        self.action_map = {
            'f': 'stop',
            'r': 'recovery_stand',
            'z': 'stand_up',
            'c': 'stand_down'
        }
      
    def _keyboard_listener(self):
        try:
            while self.running:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    
                    # 状态切换命令
                    if key in self.state_map:
                        self.command_handler.add_command(
                            Command(CommandType.STATE, self.state_map[key]))
                    
                    # 移动控制命令
                    elif key in self.movement_map:
                        self.command_handler.add_command(
                            Command(CommandType.MOVE, self.movement_map[key]))
                    
                    # 动作控制命令
                    elif key in self.action_map:
                        self.command_handler.add_command(
                            Command(CommandType.ACTION, self.action_map[key]))
                            
                    # Ctrl+C 退出
                    elif key == '\x03':
                        self.running = False
                        break
                    
                rospy.sleep(0.01)  # 添加一个小的延时,避免CPU占用过高
        
        except Exception as e:
            rospy.logerr(f"键盘输入错误: {e}")
            
    def print_instructions(self):
        print("=" * 50)
        print("""键盘控制说明：
        --- 状态切换 ---
        1: 切换到PASSIVE状态（阻尼急停）
        2: 切换到FIXEDSTAND状态（固定站立）
        3: 切换到HIGHCMD状态（高层控制）
        4: 切换到NAVIGATE状态（导航）
        
        --- 高层控制 ---
        w/s: 前进/后退
        a/d: 左移/右移
        q/e: 左转/右转
        f: 停止
        r: 恢复站立
        z: 站立（电机锁定）
        c: 趴下
        
        按Ctrl+C退出程序""")
        print("=" * 50)
            
    def stop(self):
        """停止键盘监听并恢复终端设置"""
        self.running = False
        if self.keyboard_thread.is_alive():
            self.keyboard_thread.join(timeout=1.0)
        termios.tcsetattr(sys.stdin, termios.TCSANOW, self.old_settings)
