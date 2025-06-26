#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.obstacle_detected = False
        self.twist = Twist()
        self.get_logger().info("Obstacle Avoider Node has been started.")

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)

        ranges = np.where(np.isinf(ranges), msg.range_max, ranges)
        front_angles = 60
        angle_increment = len(ranges) // 360
        front_start = len(ranges) - (front_angles // 2)*angle_increment
        front_end = (front_angles // 2)*angle_increment

        front_ranges = np.concatenate([ranges[front_start:], ranges[:front_end]])
        min_front_distance = np.min(front_ranges)

        if min_front_distance < 0.5:
            self.obstacle_detected = True
        else: 
            self.obstacle_detected = False
        
    def timer_callback(self):
        if self.obstacle_detected:
            self.twist.linear.x = 0.0
            self.twist.angular.z = -0.5
        else:
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0.0  

        self.publisher.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoider = ObstacleAvoider()

    try:
        rclpy.spin(obstacle_avoider)
    except KeyboardInterrupt:
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0   
        obstacle_avoider.publisher.publish(stop_msg)
    finally:
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        obstacle_avoider.publisher.publish(stop_msg)
        obstacle_avoider.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    