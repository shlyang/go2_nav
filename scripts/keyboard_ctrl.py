#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Twist
from atom_action import AtomicAction

class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)
        self.atomic_action = AtomicAction()
        
    def get_keyboard_input(self):
        pass

    def cmd_vel_callback(self, twist):
        print("cmd_vel_callback")
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
            elif ch == 'r':
                self.atomic_action.recovery_stand()
            elif ch == 'z':
                self.atomic_action.stand_up()
            elif ch == 'x':
                self.atomic_action.balance_stand()
            elif ch == 'c':
                self.atomic_action.stand_down()
            elif ch == 'v':
                self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
            elif ch == '\x03':
                print("退出")
                self.atomic_action.stop()
                self.atomic_action.stand_down()
                self.atomic_action.avoid_client.UseRemoteCommandFromApi(False)
                break


if __name__ == '__main__':
    try:
        control = KeyboardControl()
        control.run()
    except rospy.ROSInterruptException:
        pass