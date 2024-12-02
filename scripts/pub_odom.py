import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformStamped


class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__("odom_tf_publisher")
        self.broadcaster = TransformBroadcaster(self)
        self.odom_topic = "/utlidar/robot_odom"
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        print("subscribe" + self.odom_topic)

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform([t])
        

def main():
    rclpy.init()
    odom_tf_publisher = OdomTFPublisher()
    # loop_rate = rclpy.create_rate(10)
    # while rclpy.ok():
    #     rclpy.spin_some(odom_tf_publisher)
    #     loop_rate.sleep()
    rclpy.spin(odom_tf_publisher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()