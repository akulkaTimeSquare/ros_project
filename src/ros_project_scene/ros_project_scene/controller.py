import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class ControllerNode(Node):    
    def __init__(self):
        super().__init__('controller_node')
        self.wheel_radius = 0.045
        self.wheel_separation = 0.2
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        self.vel_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10
        )
        
        self.get_logger().info('Controller node initialized, waiting for /cmd_vel messages...')
    
    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        
        omega_left = (v - w * self.wheel_separation / 2.0) / self.wheel_radius
        omega_right = (v + w * self.wheel_separation / 2.0) / self.wheel_radius
        
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [omega_left, omega_right]
        
        self.vel_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
