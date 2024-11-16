#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from unitree_nav2.steve_basic_navigator import BasicNavigator
from action_msgs.msg import GoalStatus
from custom_interface.msg import NavState

class NavToPoseNode(Node):

    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(PoseStamped, "nav_to_pose", self.navigationToPose_callback, 10)
        self.pub = self.create_publisher(NavState,"nav_state",10)
        
        # 添加重试逻辑和错误处理
        try:
            self.navigator = BasicNavigator()
            self.get_logger().info('等待导航系统初始化...')
            
            initial_pose = Pose()
            initial_pose.orientation.w = 1.0
            self.navigator.setInitialPose(initial_pose)
            
            # 设置超时时间
            timeout = 20  # 20秒超时
            start_time = self.get_clock().now()
            
            while True:
                try:
                    self.navigator.waitUntilNav2Active()
                    break
                except Exception as e:
                    if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                        self.get_logger().error(f'导航系统初始化超时: {str(e)}')
                        raise
                    self.get_logger().warn('等待导航系统就绪...')
                    rclpy.sleep(1.0)
                
            self.get_logger().info('导航系统已就绪')
            
        except Exception as e:
            self.get_logger().error(f'导航系统初始化失败: {str(e)}')
            raise

    def navigationToPose_callback(self,msg):
        #self.get_logger().info('Publishing: "%s"' % msg.data)

        assert isinstance(msg, PoseStamped)
        # assert msg.header.frame_id == 'map'
        goal_pose = msg
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.goToPose(goal_pose)

        i = 0
        while not self.navigator.isNavComplete():
            #################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
                        
            if feedback and i % 5 == 0:
                # print('Estimated time of arrival: ' + '{0:.0f}'.format(
                #   Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                #   + ' seconds.')
                
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

        nav_state_msg = NavState()
        
        result = self.navigator.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            print('Goal succeeded!')
            nav_state_msg.state = "nav_succeed"
        elif result == GoalStatus.STATUS_CANCELED:
            print('Goal was canceled!')
            nav_state_msg.state = "nav_interrupt"
        elif result == GoalStatus.STATUS_ABORTED:
            print('Goal failed!')
            nav_state_msg.state = "nav_failed"
        else:
            print('Goal has an invalid return status!')
            nav_state_msg.state = "nav_missed"

        self.pub.publish(nav_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseNode("navigation_to_pose")
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()
