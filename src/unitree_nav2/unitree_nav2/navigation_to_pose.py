#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from custom_interface.msg import NavigationToPose,NavState

class NavToPoseNode(Node):

    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(NavigationToPose, "nav_to_pose", self.navigationToPose_callback, 10)
        self.pub = self.create_publisher(NavState,"nav_state",10)

    def navigationToPose_callback(self,msg):
        #self.get_logger().info('Publishing: "%s"' % msg.data)

        navigator = BasicNavigator()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.pose_x
        goal_pose.pose.position.y = msg.pose_y
        goal_pose.pose.position.z = msg.pose_z
        goal_pose.pose.orientation.x = msg.quat_x
        goal_pose.pose.orientation.y = msg.quat_y
        goal_pose.pose.orientation.z = msg.quat_z
        goal_pose.pose.orientation.w = msg.quat_w

        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
                        
            if feedback and i % 5 == 0:
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

        nav_state_msg = NavState()
        nav_state_msg.id = msg.id

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            nav_state_msg.state = "nav_true"
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            nav_state_msg.state = "nav_interrupt"
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            nav_state_msg.state = "nav_false"
        else:
            print('Goal has an invalid return status!')
            nav_state_msg.state = "nav_false"

        self.pub.publish(nav_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseNode("navigation_to_pose")
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()
