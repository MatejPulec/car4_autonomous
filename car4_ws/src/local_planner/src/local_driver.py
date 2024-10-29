#!/usr/bin/env python3
import rospy
import tf
import serial
import struct
import numpy as np
from geometry_msgs.msg import Polygon, Point32
import threading
import signal
import sys
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from sensor_msgs.msg import LaserScan

class LocalDriverNode:
    def __init__(self):

        self.range_threshold = 0.7
        self.repulsive_coef = 0.05
        self.speed = 160
        # Initialize ROS node
        rospy.init_node("local_driver")

        # Set up subscriber and publisher
        self.path_subscriber = rospy.Subscriber("/point_to_follow_angle_distance", Float64MultiArray, self.angle_distance_callback)
        self.path_subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_scann_callback)
        self.control_vector_publisher = rospy.Publisher("/control_vector", Int32MultiArray, queue_size=10)

    def angle_distance_callback(self, msg):

        self.goal_angle = msg.data[0]
        self.goal_distance = msg.data[1]

    def laser_scann_callback(self,msg):

        if not hasattr(self, 'goal_angle'):
            return

        attractive_force_vector = 100 * np.array([np.sin(self.goal_angle)*-1, np.cos(self.goal_angle)])
        attractive_force_vector = 100 * np.array([0, 1])
        repulsive_force_vector = np.array([0.0,0.0])

        if not hasattr(self, 'laser_angles'):
            self.laser_angles = np.arange(msg.angle_max, msg.angle_min -
                                     msg.angle_increment, -msg.angle_increment)
            
        for range, laser_angle in zip(msg.ranges, self.laser_angles):
            if range < self.range_threshold and range > 0.05:
                # diff of 1/2 n (1/x - 1/t)**2
                repulsive_intensity = self.repulsive_coef*(self.range_threshold-range)/(self.range_threshold*range**3)
                repulsive_delta = repulsive_intensity * np.array([np.sin(laser_angle), np.cos(laser_angle)])
                repulsive_force_vector = np.add(repulsive_force_vector,repulsive_delta)
        
        rospy.logwarn(repulsive_force_vector)
        total_vector = attractive_force_vector - repulsive_force_vector
        total_angle = np.arctan2(total_vector[0], total_vector[1])*-1

        speed, dir = self.dir_to_control(total_angle)

        control_vector = [99, 0, int(dir), int(speed), 0]
        rospy.logwarn(control_vector)
        msg = Int32MultiArray()
        msg.data = control_vector
        self.control_vector_publisher.publish(msg)
    
    # No use of lidar
    def dir_to_control(self, angle):

        if abs(angle) <= 1.6:
            speed = 127 + 30
            dir = int(np.clip(127 - 200 * angle, 1, 254))
        else:
            speed = 127 - 30
            dir = 254 if angle > 0 else 1
        return speed, dir

if __name__ == "__main__":
    try:
        node = LocalDriverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
