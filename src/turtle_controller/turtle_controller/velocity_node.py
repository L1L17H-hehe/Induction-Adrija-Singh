import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Turtle_Controller(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.get_logger().info('Turtle Controller Node has been started.')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        timer_period =0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: linear.x = %f, angular.z = %f' % (msg.linear.x, msg.angular.z))


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = Turtle_Controller()
    try:
        rclpy.spin(turtle_controller)
    except KeyboardInterrupt:
        turtle_controller.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
