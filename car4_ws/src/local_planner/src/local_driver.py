#!/usr/bin/env python3
import rospy
import tf
import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
import threading
import signal
import sys
from std_msgs.msg import Float64MultiArray, Int32MultiArray, Bool
from sensor_msgs.msg import LaserScan
from copy import deepcopy
import math
import os
import tensorflow as tf
import time


class LocalDriverNode:
    def __init__(self):
        # Potential field
        self.range_threshold = 0.75  # 0.75
        self.repulsive_coef = 0.1
        self.repulsive_coef_linear = 175  # 175

        # disparity extender
        self.disparity_threshold = 0.5
        self.distance_threshold = 0.5

        self.callback_repetitions = 0

        # neural network
        current_directory = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(
            current_directory, "../../imitation_learning/trained_model")
        self.loaded_model = tf.keras.models.load_model(model_path)
        model_class_path = os.path.join(
            current_directory, "../../imitation_learning/trained_model_class")
        self.loaded_model_class = tf.keras.models.load_model(model_class_path)

        self.previous_speed = 0
        self.speed = 127
        self.min_speed = 20
        self.max_speed = 50
        self.speed_increment = 50
        self.last_update_time = time.monotonic()

        self.sigma = np.pi/4

        self.visible_finish_flag = 0
        # Initialize ROS node
        rospy.init_node("local_driver")

        # Set up subscriber and publisher
        self.path_subscriber = rospy.Subscriber(
            "/point_to_follow_angle_distance", Float64MultiArray, self.angle_distance_callback)
        self.scan_subscriber = rospy.Subscriber(
            "/scan", LaserScan, self.potential_field_callback)
        self.control_vector_publisher = rospy.Publisher(
            "/control_vector", Int32MultiArray, queue_size=10)
        self.visible_finish_flag_subscriber = rospy.Subscriber(
            "/visible_finish_flag", Bool, self.visible_finish_flag_callback)

    def visible_finish_flag_callback(self, msg):
        self.visible_finish_flag = msg.data

    def angle_distance_callback(self, msg):

        self.goal_angle = msg.data[0]
        self.goal_distance = msg.data[1]

    def gauss_weight(self, angle):
        return np.exp(-0.5 * (angle / self.sigma) ** 2)

    # Potential field
    def potential_field_callback(self, msg):

        self.callback_repetitions += 1

        if not hasattr(self, 'goal_angle'):
            return

        attractive_force_vector = 100 * \
            np.array([np.sin(self.goal_angle), np.cos(self.goal_angle)])
        repulsive_force_vector = np.array([0.0, 0.0])

        rospy.logwarn(msg.header)

        self.laser_angles = np.arange(
            msg.angle_min - msg.angle_increment, msg.angle_max, msg.angle_increment)
        self.laser_ranges = [4 if r < 0.05 else r for r in msg.ranges]

        # self.laser_ranges, self.laser_angles = self.offset_points(self.laser_ranges, self.laser_angles, 0.25)
        self.laser_ranges = self.local_minima_with_radius(
            self.laser_ranges, 50)

        lidar_points = []
        local_minima_points = []

        for range, laser_angle in zip(self.laser_ranges, self.laser_angles):
            x, y = range * np.sin(laser_angle), range * np.cos(laser_angle)
            lidar_points.append((x, y))

            if range < self.range_threshold and range > 0.05:
                repulsive_intensity = (self.range_threshold - range) * \
                    (self.repulsive_coef_linear / self.range_threshold) * -1
                repulsive_delta = repulsive_intensity * \
                    np.array([np.sin(laser_angle), np.cos(laser_angle)])
                repulsive_force_vector = np.add(
                    repulsive_force_vector, repulsive_delta)
                local_minima_points.append((x, y))

        total_vector = attractive_force_vector + repulsive_force_vector
        total_angle = np.arctan2(total_vector[0], total_vector[1])

        forward, dir = self.dir_to_control(total_angle)
        target_speed = int(127 + 30 * forward)

        current_update_time = time.monotonic()
        time_delta = current_update_time - self.last_update_time
        self.last_update_time = current_update_time

        self.speed = self.speed + \
            np.clip(target_speed - self.speed, -time_delta *
                    self.speed_increment, time_delta * self.speed_increment)

        if self.speed > 127 - self.min_speed and self.speed < 127 + self.min_speed:
            if forward == 1:
                self.speed = 127 + self.min_speed
            else:
                self.speed = 127 - self.min_speed

        control_vector = [99, 0, int(dir), int(127 + forward * 30), 0]
        msg = Int32MultiArray()
        msg.data = control_vector
        self.control_vector_publisher.publish(msg)

        self.callback_repetitions += 1
        # Visualization
        if self.callback_repetitions % 20 == 0:
            plt.clf()
            lidar_x, lidar_y = zip(*lidar_points)
            minima_x, minima_y = zip(
                *local_minima_points) if local_minima_points else ([], [])
            lidar_x = np.array(lidar_x)
            lidar_y = np.array(lidar_y)
            minima_x = np.array(minima_x)
            minima_y = np.array(minima_y)

            plt.scatter(-1 * minima_x, minima_y, color='red', label='Local Minima')

            plt.quiver(0, 0, -1 * attractive_force_vector[0] * 0.01, attractive_force_vector[1] * 0.01,
                    color='green', angles='xy', scale_units='xy', scale=1, label='Attractive Force')
            plt.quiver(0, 0, -1 * repulsive_force_vector[0] * 0.01, repulsive_force_vector[1] * 0.01,
                    color='purple', angles='xy', scale_units='xy', scale=1, label='Repulsive Force')
            plt.quiver(0, 0, -1 * total_vector[0] * 0.01, total_vector[1] * 0.01, color='black',
                    angles='xy', scale_units='xy', scale=1, label='Resultant Force')

            plt.xlim(-5, 5)
            plt.ylim(-5, 5)
            plt.xlabel("X (meters)")
            plt.ylabel("Y (meters)")
            plt.title("Potential Field Visualization")
            plt.legend()
            plt.grid()
            plt.pause(0.001)

    def disparity_extender_callback(self, msg):

        self.callback_repetitions += 1

        # self.goal_angle = 0

        if not hasattr(self, 'goal_angle'):
            return

        self.laser_angles = np.arange(
            msg.angle_min - msg.angle_increment, msg.angle_max, msg.angle_increment)

        # Retrieve raw LIDAR data
        self.laser_ranges = [4 if r < 0.05 or r > 4 else r for r in msg.ranges]

        self.laser_ranges = np.array(self.laser_ranges)  # [::-1]

        self.laser_ranges, self.laser_angles = self.offset_points(
            self.laser_ranges, self.laser_angles, 0.33)

        self.laser_ranges, self.laser_angles = self.filter_angles(
            self.laser_ranges, self.laser_angles)

        # now check witch ones are ok

        # get ranges
        ranges = deepcopy(self.laser_ranges)
        masked_out_ranges = deepcopy(ranges)
        # find disparities
        disparity_TH = 0.4     # I hope its in meters
        car_safety_bbl = 0.3    # I really hope its meters, should haft of car width

        for idx, rng in enumerate(ranges):

            # to the right
            if idx < len(ranges)-1:  # gives error for the last element
                if (ranges[idx+1] - ranges[idx]) > disparity_TH:  # checkes for disparities
                    # calculates the angle for the safety bubble based on distance
                    angle = 2 * \
                        math.asin((car_safety_bbl/2) /
                                  (rng))

                    # # calculates number of laser samples
                    # mask_N = math.ceil(angle / msg.angle_increment)

                    # # decreases the range to rng
                    # for i in range(idx, min(len(ranges)-1, idx + mask_N)):
                    #     if masked_out_ranges[i] > rng:
                    #         masked_out_ranges[i] = rng

                    current_angle = self.laser_angles[idx]
                    i = 0

                    while idx + i < len(self.laser_angles) and self.laser_angles[idx + i] < current_angle + angle:
                        if masked_out_ranges[idx + i] > rng:
                            masked_out_ranges[idx + i] = rng
                        i += 1

            # to the left
            if idx > 0:  # gives error for first element
                if (ranges[idx-1] - ranges[idx]) > disparity_TH:
                    angle = 2 * \
                        math.asin((car_safety_bbl/2) /
                                  (rng))

                    # mask_N = math.ceil(angle / msg.angle_increment)

                    # for i in range(max(0, idx - mask_N), idx):
                    #     if masked_out_ranges[i] > rng:
                    #         masked_out_ranges[i] = rng

                    current_angle = self.laser_angles[idx]
                    i = 0

                    while idx - i >= 0 and self.laser_angles[idx - i] > current_angle - angle:
                        if masked_out_ranges[idx - i] > rng:
                            masked_out_ranges[idx - i] = rng
                        i += 1

        candidate_angles = set()

        # Next, search for disparities and add those angles
        for idx in range(len(masked_out_ranges) - 1):
            if abs(masked_out_ranges[idx + 1] - masked_out_ranges[idx]) > disparity_TH:
                # Choose the angle corresponding to the greater distance
                if masked_out_ranges[idx + 1] > masked_out_ranges[idx]:
                    candidate_angles.add(
                        (self.laser_angles[idx + 1], masked_out_ranges[idx + 1]))
                else:
                    candidate_angles.add(
                        (self.laser_angles[idx], masked_out_ranges[idx]))

        if self.visible_finish_flag == True:

            # First, consider angles with sufficient range
            for idx, masked_range in enumerate(masked_out_ranges):
                if masked_range >= self.goal_distance:  # Check if the distance exceeds the threshold
                    candidate_angles.add(
                        (self.laser_angles[idx], masked_range))

            if len(candidate_angles) == 0:
                candidate_angles = [(angle, rng) for angle, rng in zip(
                    self.laser_angles, masked_out_ranges)]

            candidate_angles = sorted(candidate_angles, key=lambda x: x[0])

            # Compute the angular distance to the goal angle for each candidate
            angular_distances = [abs(angle - self.goal_angle)
                                 for angle, _ in candidate_angles]

            # Select the candidate with the smallest angular distance to the goal
            best_angle = list(candidate_angles)[
                np.argmin(angular_distances)][0]

            forward, dir = self.dir_to_control(best_angle)

            target_speed = 127 + self.min_speed
            if forward == -1:
                target_speed = 127 - self.min_speed

        else:

            # First, consider angles with sufficient range
            for idx, masked_range in enumerate(masked_out_ranges):
                if masked_range > self.distance_threshold:  # Check if the distance exceeds the threshold
                    candidate_angles.add(
                        (self.laser_angles[idx], masked_range))

                if len(candidate_angles) == 0:
                    candidate_angles = set((angle, rng) for angle, rng in zip(
                        self.laser_angles, masked_out_ranges))

            candidate_angles = sorted(candidate_angles, key=lambda x: x[0])

            # Attempt at using weights
            max_value = -np.inf
            for angle, masked_range in candidate_angles:
                value = masked_range * \
                    ((self.gauss_weight(angle - self.goal_angle))
                     * self.gauss_weight(angle)**0)
                if value > max_value:
                    best_angle = angle
                    max_value = value

            forward, dir = self.dir_to_control(best_angle)

            free_space_sum = 0
            for idx, masked_range in enumerate(masked_out_ranges):
                free_space_sum = free_space_sum + masked_range * \
                    (self.gauss_weight(
                        self.laser_angles[idx])*self.gauss_weight(self.laser_angles[idx]-self.goal_angle))
            free_space_sum = free_space_sum/len(masked_out_ranges)

            target_speed = 127 + \
                np.clip(free_space_sum * self.max_speed,
                        self.min_speed, self.max_speed)

            if forward == -1:
                target_speed = 127 - self.min_speed

        current_update_time = time.monotonic()
        time_delta = current_update_time - self.last_update_time
        self.last_update_time = current_update_time

        self.speed = self.speed + \
            np.clip(target_speed-self.speed, -time_delta *
                    self.speed_increment, time_delta*self.speed_increment)

        if self.speed > 127-self.min_speed and self.speed < 127+self.min_speed:
            if forward == 1:
                self.speed = 127 + self.min_speed
            else:
                self.speed == 127 - self.min_speed

        control_vector = [99, 0, int(dir), int(127+forward*30), 0]
        msg_to_send = Int32MultiArray()
        msg_to_send.data = control_vector
        self.control_vector_publisher.publish(msg_to_send)

        if self.callback_repetitions % 10 == 1:

            plt.clf()  # Clear the previous plot

            # Convert polar coordinates to Cartesian
            lidar_x = masked_out_ranges * -np.sin(self.laser_angles)
            lidar_y = masked_out_ranges * np.cos(self.laser_angles)
            plt.plot(lidar_x, lidar_y, label='Masked Ranges', color='blue')

            best_angle_x = 4*-np.sin(best_angle)
            best_angle_y = 4*np.cos(best_angle)
            plt.plot([0, best_angle_x], [0, best_angle_y])

            # Set plot limits
            plt.xlim(-4, 4)
            plt.ylim(-4, 4)

            plt.xlabel('X (meters)')
            plt.ylabel('Y (meters)')
            plt.title('LIDAR Masked Ranges with Candidate Angles')
            plt.grid(True)
            plt.legend()
            plt.pause(0.001)

    def disparity_extender_and_potential_field_callback(self, msg):
        if not hasattr(self, 'goal_angle'):
            return
        if np.abs(self.goal_angle) > np.pi * 0.6:
            self.potential_field_callback(msg)
        else:
            self.disparity_extender_callback(msg)

    def neural_network_callback(self, msg):
        if not hasattr(self, 'goal_angle'):
            return
        laser_ranges = np.array(msg.ranges)
        input_data = np.concatenate(
            (laser_ranges, [self.goal_angle, self.goal_distance]))
        input_min = np.concatenate((np.zeros(682), [-np.pi, 0]))
        input_max = np.concatenate((np.ones(682)*4, [np.pi, 3]))
        input_data_normalized = (input_data - input_min) / \
            (input_max - input_min)  # Normalize to [0, 1]
        input_data_normalized = np.expand_dims(
            input_data_normalized, axis=0)  # model expects inputs in batches
        output = self.loaded_model.predict(input_data_normalized)
        speed_stick = output[0][0]
        turning_stick = output[0][1]
        control_vector = [99, 0, np.clip(
            int(128+(turning_stick*256/2)), 1, 254), int(127+speed_stick*-30), 0]
        msg_to_send = Int32MultiArray()
        msg_to_send.data = control_vector
        self.control_vector_publisher.publish(msg_to_send)

    def neural_network_class_callback(self, msg):
        if not hasattr(self, 'goal_angle'):
            return
        laser_ranges = np.array(msg.ranges)
        input_data = np.concatenate(
            (laser_ranges[::-1], [self.goal_angle, self.goal_distance]))
        input_min = np.concatenate((np.zeros(682), [-np.pi, 0]))
        input_max = np.concatenate((np.ones(682)*4, [np.pi, 3]))
        input_data_normalized = (input_data - input_min) / \
            (input_max - input_min)  # Normalize to [0, 1]
        input_data_normalized = np.expand_dims(
            input_data_normalized, axis=0)  # model expects inputs in batches
        output = self.loaded_model_class.predict(input_data_normalized)

        predicted_class = np.argmax(output[0])
        if predicted_class == 0:
            turning_stick = 0
            if output[0][1] > output[0][2]:
                turning_stick = output[0][1]/(output[0][0]+output[0][1]) * -2
            if output[0][1] < output[0][2]:
                turning_stick = output[0][2]/(output[0][0]+output[0][2]) * 2
            speed_stick = -1

        if predicted_class == 1:
            turning_stick = -1
            speed_stick = -1

        if predicted_class == 2:
            turning_stick = 1
            speed_stick = -1

        if predicted_class == 3:
            turning_stick = 1
            speed_stick = 1

        if predicted_class == 4:
            turning_stick = -1
            speed_stick = 1

        control_vector = [99, 0, np.clip(
            int(128+(turning_stick*256/2)), 1, 254), int(127+speed_stick*-30), 0]
        msg_to_send = Int32MultiArray()
        msg_to_send.data = control_vector
        self.control_vector_publisher.publish(msg_to_send)

    def dir_to_control(self, angle):

        if abs(angle) <= 1.6:
            # speed = 127 + 30\
            forward = 1
            dir = int(np.clip(127 - 350 * angle, 1, 254))
        else:
            # speed = 127 - 30
            forward = -1
            dir = 254 if angle > 0 else 1
        return forward, dir

    def local_minima_with_radius(self, arr, r):
        arr = list(arr)
        n = len(arr)
        result = [4] * n  # Initialize result with zeros

        for i in range(n):
            # Define the start and end indices to check within radius r
            start = max(0, i - r)
            end = min(n, i + r + 1)

            # Exclude the current element and check if it's the smallest in the neighborhood

            if arr[i] <= min(arr[start:i], default=float('inf')) and arr[i] < min(arr[i+1:end], default=float('inf')):
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

    def filter_angles(self, laser_ranges, laser_angles):
        # Find index of the angle closest to zero
        zero_index = np.argmin(np.abs(laser_angles))  # 341/342

        # Process negative to zero (strictly increasing)
        mask = np.zeros(len(laser_angles), dtype=bool)
        prev_angle = float('-inf')
        for i in range(zero_index + 1):
            if laser_angles[i] > prev_angle:  # Ensure strictly increasing
                mask[i] = True
                prev_angle = laser_angles[i]

        # Process positive to zero (strictly decreasing)
        prev_angle = float('inf')
        for i in range(len(laser_angles) - 1, zero_index, -1):
            if laser_angles[i] < prev_angle:  # Ensure strictly decreasing
                mask[i] = True
                prev_angle = laser_angles[i]

        return laser_ranges[mask], laser_angles[mask]


if __name__ == "__main__":
    try:
        node = LocalDriverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
