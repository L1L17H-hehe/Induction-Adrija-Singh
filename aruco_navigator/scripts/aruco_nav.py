#!/usr/bin/env python3 

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from cv_bridge import CvBridge
import cv2
import time 
import math

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ArucoNavigator (Node): 

    def __init__(self): 
        super().__init__('ArucoNavigator')
        self.image_subscriber = self.create_subscription(Image,'/camera/image_raw', self.image_listener_callback,10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_listener_callback,10)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.camera_matrix = np.array([[554.254691191187,0,320.5],
                                       [0,554.254691191187,240.5],
                                       [0,0,1]])
        self.distortion_coeffs = np.zeros((5,1))
        self.marker_length = 0.1
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.linear_speed = 0.2
        self.angular_speed = 0.2

        self.aruco_tags_detected = 0
        self.aruco_tags_total = 6

        self.status = "FORWARD"
        self.starting_yaw = None 
        self.current_yaw = None
        self.turn_direction = None

        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Node started...")


    def image_listener_callback(self, msg):
        if self.aruco_tags_detected < self.aruco_tags_total:
            self.get_logger().info(f"Number of tags remaining to be detected {self.aruco_tags_total-self.aruco_tags_detected-1}")
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict,parameters = self.aruco_params)

            if ids is not None: 
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.distortion_coeffs)
                ids_list = ids.flatten()
                for i, tag_id in enumerate(ids_list):
                    tvec = tvecs[i][0]
                    distance = np.linalg.norm(tvec)
                    if distance <= 0.27 and (tag_id in [0,1]):
                        if self.status == "FORWARD":
                            self.get_logger().info(f"Detected tag ID {tag_id}")
                            self.start_turn(tag_id)
                            self.aruco_tags_detected += 1
                            break #to only act on 1 tag per frame
        elif self.status == "TURNING" or self.aruco_tags_detected >= self.aruco_tags_total:
            self.get_logger().info("All tags detected, stopping robot.")
            self.stop_robot()
            return
    
    def odom_listener_callback(self,msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2*(q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2*(q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def start_turn(self, tag_id): 
        if self.current_yaw is None:
            self.get_logger().info("No yaw, can't turn")
            return
        
        self.status = "TURNING"
        self.turn_direction = "right" if tag_id == 0 else "left"
        self.starting_yaw = self.current_yaw
        self.get_logger().info(f"Starting turn {self.turn_direction} 90 degrees")

        
    def stop_robot(self): 
        twist = Twist()
        self.get_logger().info("Command recieved to stop the robot, stopping robot now.")
        twist.linear.x = 0.0
        twist.angular.z = 0.0 
        self.publisher.publish(twist)
        

        
    def timer_callback(self):
        twist = Twist()

        if (self.aruco_tags_detected >= self.aruco_tags_total):
            self.stop_robot()
            return
        
        if self.status == "TURNING":
            if self.current_yaw is None or self.starting_yaw is None:
                self.get_logger().info("Missing yaw")
                return 
            
            angle_turned = normalize_angle(self.current_yaw - self.starting_yaw)
            target_angle = (math.pi / 2)*0.95
            if self.turn_direction == "right":
                angle_turned = -angle_turned
            if abs(angle_turned) >= target_angle:
                self.status = "FORWARD"
                self.starting_yaw = None
                self.turn_direction = None
                self.get_logger().info("Turn completed, going forward")
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed if self.turn_direction == "right" else self.angular_speed
            
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        
        self.publisher.publish(twist)



def main(args = None):
    rclpy.init(args = args)
    node = ArucoNavigator()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        node.get_logger().info("Executing keyboard interrupt stopping")
        node.stop_robot()
        time.sleep(1)
    finally: 
        node.stop_robot()
        time.sleep(1)
        node.get_logger().info("Final stop maneuver of the robot before node destruction")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()