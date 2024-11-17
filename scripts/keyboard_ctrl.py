#!/usr/bin/env python3
import sys
import tty
import termios
import rospy
from geometry_msgs.msg import Twist
from atom_action import AtomAction
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)
        self.atomic_action = AtomAction()
        
    def get_keyboard_input(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def cmd_vel_callback(self, twist):
        vx = twist.linear.x
        vy = twist.linear.y
        wz = twist.angular.z
        self.atomic_action.move_with_avoid(vx, vy, wz)

    def run(self):
        """主运行循环"""
        print("键盘控制启动:")
        print("z: 站立")
        print("c: 趴下")
        print("x: 电机解锁")
        print("w/s: 前进/后退")
        print("a/d: 左移/右移")
        print("q/e: 左转/右转")
        print("v: 导航模式")
        print("f: 停止")
        print("r: 恢复站立")
        print("Ctrl+C: 退出")

        while not rospy.is_shutdown():
            ch = self.get_keyboard_input()
            
            if ch == 'w':
                self.atomic_action.move_with_avoid(0.3, 0, 0)
            elif ch == 's':
                self.atomic_action.move_with_avoid(-0.3, 0, 0)
            elif ch == 'a':
                self.atomic_action.move_with_avoid(0, 0.3, 0)
            elif ch == 'd':
                self.atomic_action.move_with_avoid(0, -0.3, 0)
            elif ch == 'q':
                self.atomic_action.move_with_avoid(0, 0, 0.3)
            elif ch == 'e':
                self.atomic_action.move_with_avoid(0, 0, -0.3)
            elif ch == 'f':
                self.atomic_action.stop()
            elif ch == 'r':
                self.atomic_action.recovery_stand()
            elif ch == 'z':
                self.atomic_action.stand_up()
            elif ch == 'x':
                self.atomic_action.balance_stand()
            elif ch == 'c':
                self.atomic_action.stand_down()
            elif ch == 'v':
                print("监听cmd_vel（ROS1）")
                self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
            elif ch == '\x03':
                print("退出")
                self.atomic_action.stop()
                self.atomic_action.stand_down()
                self.atomic_action.avoid_client.UseRemoteCommandFromApi(False)
                break


if __name__ == '__main__':
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)
    try:
        control = KeyboardControl()
        control.run()
    except rospy.ROSInterruptException:
        pass