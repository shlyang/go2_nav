from state_machine.BaseState import BaseState
from common.enums import FSMStateName, StateCmd
import rospy
from geometry_msgs.msg import Twist

class NavigateState(BaseState):
    def __init__(self, ctrl_handler):
        super().__init__(ctrl_handler, FSMStateName.NAVIGATE)
        self.vel_sub = None
        
    def enter(self):
        self._ctrl_handler.balance_stand()
        # self.listen_cmd_vel
        if self.vel_sub is None:
            self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
            rospy.loginfo("(Re)subscribed to /cmd_vel.")
        
    def run(self, cmd_handler):
        pass
    
    def exit(self):
        self.state_cmd = StateCmd.NONE
        if self.vel_sub is not None:
            self.vel_sub.unregister()
            rospy.loginfo("Stopped subscribing to /cmd_vel.")
            self.vel_sub = None
    
    def check_change(self):
        if self.state_cmd == StateCmd.PASSIVE:
            return FSMStateName.PASSIVE
        elif self.state_cmd == StateCmd.FIXEDSTAND:
            return FSMStateName.FIXEDSTAND
        else:
            return FSMStateName.NAVIGATE
        
    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        rospy.loginfo(f"Received cmd_vel: {msg}")
        self._ctrl_handler.move(vx, vy, wz)
        