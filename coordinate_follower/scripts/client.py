#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from coordinate_follower.action import FollowCoordinate as Chase
import os

class CoordinateFollowerClient(Node): 

    def __init__(self): 
        super().__init__('client')
        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, 
            Chase,
            'chase',
            callback_group=self.callback_group
        )
        self.coordinates = self._read_coordinates()
        self.goal_idx = 0
        self.done = False

        self.get_logger().info("Waiting for action server")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server connected")
        if self.coordinates:
            self._send_goal()
        else:
            self.get_logger().info("No valid goals found in coordinate file.")
            self.done = True

    def _read_coordinates(self):
        coordinates = []
        path = '/home/l1l17h/ros2_ws/src/coordinate_follower/coordinates.txt'
        if not os.path.exists(path):
            self.get_logger().warn(f"Coordinate file not found at {path}")
            return []
        with open(path, 'r') as f: 
            for line in f: 
                part = line.strip()
                if not part or part.startswith('#'):
                    continue
                parts = part.split()
                if len(parts) != 2:
                    self.get_logger().warn(f"Skipping malformed line: '{line.strip()}'")
                    continue
                try:
                    x, y = map(float, parts)
                    coordinates.append((x, y))
                except ValueError: 
                    self.get_logger().warn(f"Skipping non-numeric line: '{line.strip()}'")
        return coordinates
        
    def _send_goal(self): 
        if self.goal_idx >= len(self.coordinates): 
            self.get_logger().info("All goals done")
            self.done = True
            return 
        x,y = self.coordinates[self.goal_idx]
        goal_msg = Chase.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        self.get_logger().info(f"Sending goal: ({x}, {y})")

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.done = True
            return
        self.get_logger().info("Goal accepted") 
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback 
        self.get_logger().info(
            f'Current: ({feedback.current_x:.2f}, {feedback.current_y:.2f}), '
            f'Distance: {feedback.distance_to_goal:.2f}'
        )

    def _get_result_callback(self, future):
        result = future.result().result
        if getattr(result, 'success', False):
            self.get_logger().info("Goal accomplished")
            self.goal_idx += 1
            self._send_goal()
        else:
            self.get_logger().info(f'Goal failed at: ({getattr(result, "final_x", "N/A")}, {getattr(result, "final_y", "N/A")})')
            self.done = True

def main(args=None):
    rclpy.init(args=args)
    client = CoordinateFollowerClient()
    if not client.coordinates:
        client.get_logger().info("No goals, dying...")
        client.destroy_node()
        rclpy.shutdown()
        return 
    
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    try:
        while rclpy.ok() and not client.done:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt: 
        client.get_logger().info("Interrupt Detected") 
    finally: 
        executor.remove_node(client)
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': 
    main()
    