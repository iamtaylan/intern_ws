#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MovementController:
    def __init__(self):
        rospy.init_node('movement_node', anonymous=True)

        # parameters
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)
        self.angular_speed = rospy.get_param('~angular_speed', 0.3)
        self.move_duration = rospy.get_param('~move_duration', 5.0)

        # Variables
        self.current_yaw = 0.0
        self.current_pose = {'x': 0.0, 'y': 0.0}
        
        # Publisher/Subscriber
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.rate = rospy.Rate(20)

    # Updates the robot's current position and orientation by processing incoming odometry data.
    def odom_callback(self, msg):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(quaternion)
        self.current_yaw = yaw

        # Output:
        rospy.loginfo("X: {:.2f} | Y: {:.2f} | Yaw: {:.2f} rad".format(
            self.current_pose['x'], self.current_pose['y'], self.current_yaw))

    # Ensures angular continuity by constraining the angle within the range of -PI to +PI.
    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    # Rotates the robot using angular error control until the target orientation is reached.
    def rotate_90_degrees(self):
        vel_msg = Twist()
        target_rad = 1.57   # 90-Degree
        
        start_yaw = self.current_yaw
        target_yaw = self.normalize_angle(start_yaw + target_rad)
        
        rospy.loginfo("Starting Rotation. Target Yaw: {:.2f}".format(target_yaw))

        while not rospy.is_shutdown():
            angle_error = self.normalize_angle(target_yaw - self.current_yaw)
            
            if -0.01 < angle_error < 0.01:
                break
            
            vel_msg.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Rotation completed.")

    # Executes the sequence of tasks sequentially.
    def run_task(self):
        # 1. Forward Motion
        rospy.loginfo("Stage 1: Moving forward...")
        start_time = rospy.get_time()
        vel_msg = Twist()
        
        while rospy.get_time() - start_time < self.move_duration:
            vel_msg.linear.x = self.linear_speed
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # 2. Stop
        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.sleep(1.0)

        # 3. 90-Degree Rotation 
        rospy.loginfo("Stage 2: Rotating 90 degrees...")
        self.rotate_90_degrees()

        # 4. Stop
        rospy.loginfo("All tasks completed successfully.")
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MovementController()
        rospy.sleep(1.0) 
        controller.run_task()
    except rospy.ROSInterruptException:
        pass