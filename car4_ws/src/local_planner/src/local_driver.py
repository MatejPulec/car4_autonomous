#!/usr/bin/env python3
import rospy
import tf
import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Polygon, Point32
import threading
import signal
import sys
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from sensor_msgs.msg import LaserScan
from copy import deepcopy
import math

class LocalDriverNode:
    def __init__(self):
        # Potential field
        self.range_threshold = 0.4 
        self.repulsive_coef = 4
        self.safety_bouble_radius = 0.2

        #disparity extender
        self.disparity_threshold = 0.5
        self.distance_threshold = 2

        self.speed = 180
        # Initialize ROS node
        rospy.init_node("local_driver")

        # Set up subscriber and publisher
        self.path_subscriber = rospy.Subscriber("/point_to_follow_angle_distance", Float64MultiArray, self.angle_distance_callback)
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.potential_field_callback)
        self.control_vector_publisher = rospy.Publisher("/control_vector", Int32MultiArray, queue_size=10)

    def angle_distance_callback(self, msg):

        self.goal_angle = msg.data[0]
        self.goal_distance = msg.data[1]

    ## Potential field
    def potential_field_callback(self,msg):

        if not hasattr(self, 'goal_angle'):
            return

        attractive_force_vector = 100 * np.array([np.sin(self.goal_angle)*-1, np.cos(self.goal_angle)])
        repulsive_force_vector = np.array([0.0,0.0])

        if not hasattr(self, 'laser_angles'):
            self.laser_angles = np.arange(msg.angle_max, msg.angle_min -
                                     msg.angle_increment, -msg.angle_increment)
        
        # if not hasattr(self, 'laser_angles_no_offset'):
        #     self.laser_angles_no_offset = np.arange(msg.angle_max, msg.angle_min -
        #                              msg.angle_increment, -msg.angle_increment)
            
        # self.laser_ranges = [float('nan') if r < 0.05 else r for r in msg.ranges]

        # self.laser_ranges, self.laser_angles = self.offset_points(self.laser_ranges, self.laser_angles_no_offset, 0.33)

        # self.laser_ranges = self.local_minima_with_radius(self.laser_ranges, 20)

        # self.laser_ranges = np.array(self.laser_ranges)
        # self.laser_ranges -= self.safety_bouble_radius
        
        # for range, laser_angle in zip(self.laser_ranges, self.laser_angles):

        for range, laser_angle in zip(msg.ranges, self.laser_angles):
            if range < self.range_threshold and range > 0.05:
                # diff of 1/2 n (1/x - 1/t)**2
                repulsive_intensity = self.repulsive_coef*(self.range_threshold-range)/(self.range_threshold*range**3)
                repulsive_delta = repulsive_intensity * np.array([np.sin(laser_angle), np.cos(laser_angle)])
                repulsive_force_vector = np.add(repulsive_force_vector,repulsive_delta)
        
        total_vector = attractive_force_vector - repulsive_force_vector
        total_angle = np.arctan2(total_vector[0], total_vector[1])*-1

        speed, dir = self.dir_to_control(total_angle)

        control_vector = [99, 0, int(dir), int(speed), 0]
        msg = Int32MultiArray()
        msg.data = control_vector
        self.control_vector_publisher.publish(msg)


    def laser_scan_callback(self, msg):

        # self.goal_angle = 0

        if not hasattr(self, 'goal_angle'):
            return
        
        if not hasattr(self, 'laser_angles'):
            self.laser_angles = np.arange(msg.angle_max, msg.angle_min -
                                     msg.angle_increment, -msg.angle_increment)
            
        # Retrieve raw LIDAR data
        laser_ranges = np.array(msg.ranges)

        laser_ranges = [10 if r < 0.05 else r for r in msg.ranges]

        #get ranges
        ranges = deepcopy(laser_ranges)
        masked_out_ranges = deepcopy(ranges)
        #find disparities
        disparity_TH = 0.5     # I hope its in meters
        car_safety_bbl = 0.4    # I really hope its meters
        
        for idx, rng in enumerate(ranges):
            
            # to the right
            if idx < len(ranges)-1: #gives error for the last element
                if (ranges[idx+1] - ranges[idx])>disparity_TH :  #checkes for disparities
                    angle = 2 * math.asin((car_safety_bbl/2)/max(rng, car_safety_bbl/2))   #calculates the angle for the safety bubble based on distance
                    
                    mask_N = math.ceil(angle / msg.angle_increment)    #calculates number of laser samples
                    
                    for i in range(idx, min(len(ranges)-1, idx + mask_N)):  #decreases the range to rng
                        if masked_out_ranges[i] > rng:
                            masked_out_ranges[i] = rng
                        
            # to the left
            if idx > 0: #gives error for first element
                if (ranges[idx-1] - ranges[idx])>disparity_TH :
                    angle = 2 * math.asin((car_safety_bbl/2)/max(rng, car_safety_bbl/2))
                    
                    mask_N = math.ceil(angle / msg.angle_increment)
                    
                    for i in range(max(0, idx - mask_N), idx):
                        if masked_out_ranges[i] > rng:
                            masked_out_ranges[i] = rng
                        
            
        # Identify candidate angles

        candidate_angles = []

        # First, consider angles with sufficient range
        for idx, masked_range in enumerate(masked_out_ranges):
            if masked_range > self.distance_threshold:  # Check if the distance exceeds the threshold
                candidate_angles.append(self.laser_angles[idx])

        # Next, search for disparities and add those angles
        for idx in range(len(masked_out_ranges) - 1):
            if abs(masked_out_ranges[idx + 1] - masked_out_ranges[idx]) > disparity_TH:
                # Choose the angle corresponding to the greater distance
                if masked_out_ranges[idx + 1] > masked_out_ranges[idx]:
                    candidate_angles.append(self.laser_angles[idx + 1])
                else:
                    candidate_angles.append(self.laser_angles[idx])

        # Remove duplicates and sort candidate angles
        candidate_angles = sorted(set(candidate_angles))

        # Choose the best candidate angle
        if candidate_angles:
            # Compute the angular distance to the goal angle for each candidate
            angular_distances = [abs(angle - self.goal_angle) for angle in candidate_angles]
            # Select the candidate with the smallest angular distance to the goal
            best_angle = candidate_angles[np.argmin(angular_distances)]

        
        speed, dir = self.dir_to_control(-best_angle)

        control_vector = [99, 0, int(dir), int(speed), 0]
        msg = Int32MultiArray()
        msg.data = control_vector
        self.control_vector_publisher.publish(msg)

        # idx_direction = masked_out_ranges.index(max_masked_out)
                        
        # steering_angle = (msg.angle_min + (idx_direction/(len(masked_out_ranges)-1))*(msg.angle_max - msg.angle_min)) #go in direction of max value
    
    def dir_to_control(self, angle):

        if abs(angle) <= 1.6:
            speed = 127 + 30
            dir = int(np.clip(127 - 200 * angle, 1, 254))
        else:
            speed = 127 - 30
            dir = 254 if angle > 0 else 1
        return speed, dir
    
    def local_minima_with_radius(self, arr, r):
        arr = list(arr)
        n = len(arr)
        result = [0] * n  # Initialize result with zeros
        
        for i in range(n):
            # Define the start and end indices to check within radius r
            start = max(0, i - r)
            end = min(n, i + r + 1)
            
            # Exclude the current element and check if it's the smallest in the neighborhood

            if arr[i] < min(arr[start:i] + arr[i+1:end]):
                result[i] = arr[i]
    
        return result

    def offset_points(self, distances, angles, x_offset):
        # Convert polar coordinates to Cartesian
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)

        # Apply the x-direction offset
        x += x_offset

        # Convert back to polar coordinates
        offset_distances = np.sqrt(x**2 + y**2)
        offset_angles = np.arctan2(y, x)

        return offset_distances, offset_angles
    

if __name__ == "__main__":
    try:
        node = LocalDriverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
