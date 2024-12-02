import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetBool

class Go2BehaviorControl(Node):

    def __init__(self):
        super().__init__('go2_behavior_control')
        
        self.pub = self.create_publisher(String, 'robot_behavior_topic', 10)
        
        self.voice_sub = self.create_subscription(
            String,
            'robot_command_topic',
            self.voice_callback,
            10)
        
        # Service
        self.srv = self.create_service(SetBool, 'set_robot_behavior', self.set_behavior_callback)
        
        # 待检测目标
        self.object_label = 'glove'
        
        # State machine initialization
        self.state = 'IDLE'
        self.get_logger().info('Go2 Behavior Control Node has been started.')

    def voice_callback(self, msg):
        self.get_logger().info('Received command: "%s"' % msg.data)
        # Process the command and update state
        if msg.data == 'START':
            self.state = 'RUNNING'
        elif msg.data == 'STOP':
            self.state = 'IDLE'
        self.publish_state()

    def set_behavior_callback(self, request, response):
        if request.data:
            self.state = 'ACTIVE'
            response.success = True
            response.message = 'Robot behavior set to ACTIVE'
        else:
            self.state = 'INACTIVE'
            response.success = True
            response.message = 'Robot behavior set to INACTIVE'
        self.publish_state()
        return response

    def publish_state(self):
        msg = String()
        msg.data = f'Current state: {self.state}'
        self.publisher_.publish(msg)
        self.get_logger().info('Published state: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    go2_behavior_control = Go2BehaviorControl()
    rclpy.spin(go2_behavior_control)
    go2_behavior_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()