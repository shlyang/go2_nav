#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from custom_interface.msg import NavState

class NavToPoseNode(Node):

    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(PoseStamped, "nav_to_pose", self._navToPose_callback, 10)
        # self.pub = self.create_publisher(NavState,"nav_state",10)
        self.pub = self.create_publisher(String,"nav_state",10)
        
        
        try:
            # 直接创建导航动作客户端
            self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.get_logger().info('等待导航动作服务器...')
            
            # 等待导航服务器就绪
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('导航服务器连接超时！')
                raise RuntimeError('导航服务器不可用')
                
            self.get_logger().info('导航系统已就绪')
            
        except Exception as e:
            self.get_logger().error(f'导航系统初始化失败: {str(e)}')
            raise

    def _navToPose_callback(self, msg):
        assert isinstance(msg, PoseStamped)
        goal_pose = msg
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # 创建导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info('发送导航目标: ' + 
                               str(goal_pose.pose.position.x) + ' ' +
                               str(goal_pose.pose.position.y))
        
        # 发送导航目标
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        # rclpy.spin_until_future_complete(self, send_goal_future)
        # 等待目标被接受
        send_goal_future.add_done_callback(self._goal_response_callback)      
                
    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback:
            # foxy版本只能输出剩余距离
            self.get_logger().info(f'预计还需 {feedback.distance_remaining:.2f} 米到达目标')

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('导航目标被拒绝！')
            return
        self.get_logger().info('导航目标被接受')
        # 获取结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        # nav_state_msg = NavState()
        nav_state_msg = String()
        status = future.result().status
        print('status: ', status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('导航成功！')
            nav_state_msg.data = "nav_succeed"
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('导航被取消！')
            nav_state_msg.data = "nav_interrupt"
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info('导航失败！')
            nav_state_msg.data = "nav_failed"
        else:
            self.get_logger().info('导航状态未知！')
            nav_state_msg.data = "nav_missed"

        self.pub.publish(nav_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseNode("navigation_to_pose")
    rclpy.spin(node)
    node.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
    main()