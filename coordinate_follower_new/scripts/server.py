#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from rclpy.action import ActionServer 
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math 
import time
from coordinate_follower_new.action import FollowCoordinate as Chase
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist

class CoordinateFollowerServer(Node): 

    def __init__(self): 
        super().__init__('server')
        self.current_x = 0.0
        self.current_y = 0.0
        self.distance_tolerance = 0.1 
        self.angle_tolerance = 0.1 
        self.linear_speed = 0.5 
        self.angular_speed = 1.0
        self.direction = 0.0

        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, 
            Chase, 
            'chase', 
            execute_callback = self.execute_callback, 
            callback_group = self._callback_group
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, 
            '/odom',
            self.odom_callback,
            10, 
            callback_group = self._callback_group
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Coordinate follower server initiated")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x 
        self.current_y = msg.pose.pose.position.y

        d = msg.pose.pose.orientation
        siny = 2*(d.w * d.z + d.x * d.y)
        cosy = 1 - 2*(d.z * d.z + d.y * d.y) 
        self.direction = math.atan2(siny, cosy)

    def execute_callback(self, goal_handle): 
        goal = goal_handle.request
        self.get_logger().info('Going to goal')
        feedback_msg = Chase.Feedback()
        result = Chase.Result() 

        while rclpy.ok(): 
            if goal_handle.is_cancel_requested: 
                goal_handle.canceled() 
                self.get_logger().info("Goal cancelled")
                self._stop_bot()
                return result
            dx = goal.x - self.current_x
            dy = goal.y - self.current_y
            distance = math.sqrt(dx * dx + dy * dy)

            feedback_msg.current_x = self.current_x
            feedback_msg.current_y = self.current_y
            feedback_msg.distance_to_goal = distance
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(
                f'Current: ({self.current_x:.2f}, {self.current_y:.2f}), '
                f'Goal: ({goal.x:.2f}, {goal.y:.2f}), Distance = {distance:.2f}'
            )

            if distance <self.distance_tolerance: 
                result.success = True
                result.final_x = self.current_x
                result.final_y = self.current_y
                goal_handle.succeed()
                self.get_logger().info("Goal reached ")
                self._stop_bot()
                return result
            
            self._move_towards_goal(dx,dy,distance)
            time.sleep(0.1)
            
    def _move_towards_goal(self, dx, dy, distance):
        cmd_msg = Twist() 
        
        angle_goal = math.atan2(dy, dx) 
        error = math.atan2(math.sin(angle_goal - self.direction), math.cos(angle_goal - self.direction))

        if abs(error) > self.angle_tolerance:
            cmd_msg.linear.x = 0.0 
            cmd_msg.angular.z = 0.8*error 
        else: 
            cmd_msg.linear.x = min(self.linear_speed, distance*0.5)
            cmd_msg.angular.z = 0.3*error
        
        cmd_msg.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd_msg.angular.z))
        self.cmd_vel_pub.publish(cmd_msg)

    def _stop_bot(self): 
        stop = Twist() 
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_vel_pub.publish(stop)


def main(args = None):
    rclpy.init(args = args) 
    server = CoordinateFollowerServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try: 
        executor.spin()
    except KeyboardInterrupt: 
        server.get_logger().info("Interrupt Detected") 
    finally: 
        executor.remove_node(server)
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()


