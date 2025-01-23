#!/usr/bin/env python3
import rospy
import numpy as np
import tf
from sensor_msgs.msg import LaserScan
from PIL import Image
import os
import logging
import threading
import tf.transformations as tf_trans
import matplotlib.pyplot as plt  # Import for visualization
import pygame
import random
from geometry_msgs.msg import Point, Polygon
from std_msgs.msg import Float64MultiArray, Int32MultiArray
import signal
import pickle
import time

# Configure the logging
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(levelname)s - %(message)s')


class Laser:
    def __init__(self):
        self.angles = []

    def callback(self, msg):
        self.ranges = msg.ranges
        if len(self.angles) == 0:
            self.angles = np.arange(
                msg.angle_max, msg.angle_min - msg.angle_increment, -msg.angle_increment)


def bresenham_raytrace(start, end, map):
    """Ray tracing to simulate LIDAR scan."""
    distance = 0
    x1, y1 = start
    x2, y2 = end
    is_steep = abs(y2 - y1) > abs(x2 - x1)

    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    if x2 > x1:
        x_step = 1
    else:
        x_step = -1
    if y2 > y1:
        y_step = 1
    else:
        y_step = -1

    abs_dx = np.abs(x2 - x1)
    abs_dy = np.abs(y2 - y1)

    x = x1
    y = y1
    D = 2 * abs_dy - abs_dx
    for x in range(x1, x2 + x_step, x_step):
        x_to_write = x
        y_to_write = y
        if is_steep:
            x_to_write, y_to_write = y_to_write, x_to_write
        if map[y_to_write, x_to_write] == 0:
            distance = np.linalg.norm(
                [start[0]-x_to_write, start[1]-y_to_write])
            break

        if D > 0:
            y = y + y_step
            D = D - 2 * abs_dx
        D = D + 2*abs_dy

    return distance


def generate_scan(pos, angle, map, resolution, LIDAR_NUMBER_OF_POINTS, LIDAR_ANGLE):
    """Generate lidar scan using ray tracing."""
    angles = np.linspace(-(LIDAR_ANGLE/2), (LIDAR_ANGLE/2),
                         LIDAR_NUMBER_OF_POINTS)
    angles = np.deg2rad(angles)
    distances = np.zeros(LIDAR_NUMBER_OF_POINTS)
    max_range = 4 / resolution
    compensation_angle = 0
    for i, a in enumerate(angles):
        # Calculate the direction vector based on the lidar angle
        direction = np.array(
            [np.cos(a + angle + compensation_angle), np.sin(a + angle + compensation_angle)])
        end_pos = pos + max_range * direction
        max_pos = [map.shape[1]-1, map.shape[0]-1]
        min_pos = [0, 0]
        range = []

        if end_pos[0] > max_pos[0]:
            range.append(np.abs((max_pos[0] - pos[0])/direction[0]))

        if end_pos[0] < min_pos[0]:
            range.append(np.abs((min_pos[0] - pos[0])/direction[0]))

        if end_pos[1] > max_pos[1]:
            range.append(np.abs((max_pos[1] - pos[1])/direction[1]))

        if end_pos[1] < min_pos[1]:
            range.append(np.abs((min_pos[1] - pos[1])/direction[1]))

        if len(range) != 0:
            end_pos = pos + min(range) * direction

        end_pos = np.ceil(end_pos)
        pos = np.ceil(pos)
        pos = [int(pos[0]), int(pos[1])]
        end_pos = [int(end_pos[0]), int(end_pos[1])]

        distances[i] = resolution * bresenham_raytrace(pos, end_pos, map)

    return angles, distances


def car_motion_update(position, angle, speed, turning_radius, delta_time):
    """Update the car's position and angle based on speed and turning radius."""

    turning_radius = turning_radius / resolution * -1

    if turning_radius == 0:
        # Straight line motion
        dx = speed * np.cos(angle)
        dy = speed * np.sin(angle)
        new_position = [position[0] + dx *
                        delta_time, position[1] + dy * delta_time]
        new_angle = angle
    else:
        # Circular motion
        R = turning_radius
        # Calculate the angular velocity
        angular_velocity = speed / R
        # Update position and orientation
        new_angle = angle + angular_velocity * delta_time
        local_x = R * np.sin(angular_velocity * delta_time)
        local_y = R * (1 - np.cos(angular_velocity * delta_time))
        dx = np.cos(angle) * local_x - np.sin(angle) * local_y
        dy = np.sin(angle) * local_x + np.cos(angle) * local_y
        new_position = [position[0] + dx, position[1] + dy]

    return new_position, new_angle


def update_plot(map, position, angle, speed, turning_radius):
    """Update the car's position on the map."""

    position, angle = car_motion_update(
        position, angle, speed, turning_radius, delta_time=0.1)

    # Get the lidar scan using ray tracing
    laser_angles, laser_ranges = generate_scan(
        position, angle, map, resolution, LIDAR_NUMBER_OF_POINTS, LIDAR_ANGLE)

    return position, angle, laser_angles, laser_ranges


def plot_stuff(lidar_ax, laser_angles, laser_ranges, goal_angle, goal_distance):
    """
    Plot the LIDAR scan on the second subplot and mark the goal position as a red star.

    Args:
        lidar_ax: The matplotlib axis to plot the LIDAR scan.
        laser_angles: Array of angles from the LIDAR.
        laser_ranges: Array of distances corresponding to the LIDAR angles.
        goal_angle: The angle (in radians) to the goal relative to the sensor.
        goal_distance: The distance to the goal.
    """
    import numpy as np  # Ensure NumPy is imported

    # Clear and set up the plot
    lidar_ax.clear()
    lidar_ax.set_title("LIDAR Scan")
    lidar_ax.set_xlim(-5, 5)
    lidar_ax.set_ylim(-5, 5)
    lidar_ax.set_xlabel("X")
    lidar_ax.set_ylabel("Y")

    # Convert LIDAR polar coordinates to Cartesian coordinates
    lidar_x = laser_ranges * np.sin(laser_angles)
    lidar_y = laser_ranges * np.cos(laser_angles)

    # Plot the LIDAR points
    lidar_ax.scatter(lidar_x, lidar_y, c='blue', s=2, label="LIDAR Points")

    # Convert goal polar coordinates to Cartesian coordinates
    goal_angle = goal_angle * -1
    goal_x = goal_distance * np.sin(goal_angle)
    goal_y = goal_distance * np.cos(goal_angle)

    # Plot the goal as a red star
    lidar_ax.scatter(goal_x, goal_y, c='red', marker='*', s=100, label="Goal")

    # Add a legend for clarity
    lidar_ax.legend()


def lookup(value, lookup_in, lookup_out):
    # Clip the value to the range of lookup_in to avoid extrapolation
    value = np.clip(value, lookup_in[0], lookup_in[-1])
    # Find indices for interpolation
    idx = np.searchsorted(lookup_in, value)
    if idx == 0:
        return lookup_out[0]
    if idx >= len(lookup_in):
        return lookup_out[-1]

    # Linear interpolation
    x0, x1 = lookup_in[idx - 1], lookup_in[idx]
    y0, y1 = lookup_out[idx - 1], lookup_out[idx]
    return y0 + (y1 - y0) * (value - x0) / (x1 - x0)


def publish_coordinates(x, y):
    point = Point()
    point.x = x
    point.y = y
    point.z = 0  # Assuming z = 0 for 2D map
    goal_publisher.publish(point)


def send_tf_transformation(position, angle, tf_broadcaster, resolution):
    quaternion = tf.transformations.quaternion_from_euler(
        0, 0, angle * -1)
    tf_broadcaster.sendTransform(
        # Translation
        (position[0] * resolution,
            position[1] * resolution * -1, 0),  # different axes
        quaternion,
        rospy.Time.now(),
        "base_link",  # Child frame
        "map"  # Parent frame
    )


# NOT USED, if everything works, this along with many functions can be deleted
# It uses the class instead
def main():

    rospy.init_node('car_simulation')

    global position, angle, resolution, LIDAR_ANGLE, LIDAR_NUMBER_OF_POINTS, goal_publisher

    # Setup
    LIDAR_ANGLE = 240
    LIDAR_NUMBER_OF_POINTS = 682
    goal_publisher = rospy.Publisher('/goal_coordinates', Point, queue_size=10)
    tf_broadcaster = tf.TransformBroadcaster()
    resolution = 0.025
    script_dir = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(script_dir, '../map_mashup.pgm')
    file_path_safety_bubble = os.path.join(
        script_dir, '../map_mashup_with_safety_bubble.pgm')
    lookup_in = np.array(
        [-4, -3, -2, -1.5, -0.6, -0.05, 0, 0.05, 0.6, 1.5, 2, 3, 4])
    lookup_out = np.array(
        [-0.65, -1.1, -2, -4, -10, -20, 0, 20, 10, 4, 2, 1.1, 0.65])

    # Initialize Pygame and the Joystick
    pygame.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Gamepad connected: {joystick.get_name()}")

    # Load map
    with Image.open(file_path) as img:
        map = np.array(img)
    with Image.open(file_path_safety_bubble) as img:
        map_safety_bubble = np.array(img)
    y, x = np.where(map_safety_bubble == 255)
    ok_position = list(zip(x, y))

    position = [114, 500]
    angle = 0
    send_tf_transformation(position, angle, tf_broadcaster, resolution)

    # Setup the map and lidar visualization
    fig, lidar_ax = plt.subplots(1, 1, figsize=(15, 15))
    lidar_ax.set_aspect('equal', adjustable='box')

    # Start simulation loop

    rate = rospy.Rate(100)

    goal = random.choice(ok_position)
    publish_coordinates(goal[0], goal[1])

    while not rospy.is_shutdown():
        pygame.event.pump()

        speed_stick = joystick.get_axis(1)
        turning_stick = joystick.get_axis(2)
        if abs(speed_stick) < 0.05:
            speed_stick = 0
        if abs(turning_stick) < 0.05:
            turning_stick = 0
        print(speed_stick, turning_stick)
        speed = -speed_stick * 30
        turning_radius = -lookup(turning_stick *
                                 4, lookup_in, lookup_out)

        # Update car position, map, and LIDAR data
        position, angle, laser_angles, laser_ranges = update_plot(
            map, position, angle, speed, turning_radius)

        send_tf_transformation(position, angle, tf_broadcaster, resolution)

        msg = rospy.wait_for_message(
            "/point_to_follow_angle_distance", Float64MultiArray)
        goal_angle = msg.data[0]
        goal_distance = msg.data[1]

        # Update the LIDAR plot
        plot_stuff(lidar_ax, laser_angles, laser_ranges,
                   goal_angle, goal_distance)

        # Redraw the plots
        plt.draw()
        # Pause for 100ms to control the simulation update rate
        plt.pause(0.0001)

        rate.sleep()

    # Keep the plot open
    plt.show()


class SimulationNode():
    def __init__(self):
        rospy.init_node('car_simulation')
        rospy.loginfo("Node_started")

        signal.signal(signal.SIGINT, self.handle_shutdown)

        self.training_data = []
        self.training_data_current_batch = []

        self.user_control = 1
        self.control_vector = []

        self.starting_position = [0, 0]

        # Constants
        self.LIDAR_ANGLE = 240
        self.LIDAR_NUMBER_OF_POINTS = 682
        self.resolution = 0.025

        # Publishers, subscriberst etc
        self.goal_publisher = rospy.Publisher(
            '/goal_coordinates', Point, queue_size=10)
        self.scan_publisher = rospy.Publisher(
            '/scan', LaserScan, queue_size=10)
        self.goal_subscriber = rospy.Subscriber(
            '/goal_coordinates', Point, self.goal_callback)
        self.path_subscriber = rospy.Subscriber(
            "/point_to_follow_angle_distance", Float64MultiArray, self.angle_distance_callback)
        self.control_vector_subscriber = rospy.Subscriber(
            "/control_vector", Int32MultiArray, self.control_vector_callback)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.loginfo("Waiting for the first angle/distance callback...")

        # scan prep
        self.scan = LaserScan()
        self.scan.header.stamp = rospy.Time.now()
        self.scan.header.frame_id = "lidar_frame"

        # Populate LaserScan message fields
        self.scan.angle_min = np.deg2rad(-119.885)
        self.scan.angle_max = np.deg2rad(119.885)
        self.scan.angle_increment = np.deg2rad(119.885*2/681)
        self.scan.time_increment = 0.0667/682
        self.scan.scan_time = 0.1
        self.scan.range_min = 0.02
        self.scan.range_max = 4.0

        # Map
        script_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(script_dir, '../map_mashup.pgm')
        file_path_safety_bubble = os.path.join(
            script_dir, '../map_mashup_with_safety_bubble.pgm')
        self.lookup_in = np.array(
            [-4, -3, -2, -1.5, -0.6, -0.05, 0, 0.05, 0.6, 1.5, 2, 3, 4])
        self.lookup_out = np.array(
            [-0.65, -1.1, -2, -4, -10, -20, 0, 20, 10, 4, 2, 1.1, 0.65])
        with Image.open(file_path) as img:
            self.map = np.array(img)
        with Image.open(file_path_safety_bubble) as img:
            self.map_safety_bubble = np.array(img)
        y, x = np.where(self.map_safety_bubble == 255)
        self.ok_position = list(zip(x, y))

        # take only tha hallway
        self.ok_position = [(x, y)
                            for x, y in self.ok_position if 500 <= x <= 2600]

        # Initialize Pygame and the Joystick
        if self.user_control == 1:
            pygame.init()
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            print(f"Gamepad connected: {joystick.get_name()}")

        # Initial state
        self.position = [1400, 550]
        self.starting_position = self.position
        self.angle = 0

        self.send_tf_transformation()

        self.tf_timer = rospy.Timer(
            rospy.Duration(0.1), self.send_tf_transformation)

        # Setup the map and lidar visualization
        self.fig, self.lidar_ax = plt.subplots(1, 1, figsize=(15, 15))
        self.lidar_ax.set_aspect('equal', adjustable='box')

        # Start simulation loop

        self.rate = rospy.Rate(20)

        self.choose_goal()

        msg = rospy.wait_for_message(
            "/point_to_follow_angle_distance", Float64MultiArray)
        self.angle_distance_callback(msg)

        i = 0
        while not rospy.is_shutdown():
            i += 1

            if self.user_control == 1:
                pygame.event.pump()
                self.speed_stick = joystick.get_axis(1)
                self.turning_stick = joystick.get_axis(2)
            else:
                if self.control_vector == []:
                    self.speed_stick = 0
                    self.turning_stick = 0
                else:
                    self.speed_stick = (self.control_vector[3]-127)/-127
                    self.turning_stick = (self.control_vector[2]-127)/-127
                    if self.speed_stick > 0:
                        self.speed_stick = 1
                    if self.speed_stick < 0:
                        self.speed_stick = -1
            if abs(self.speed_stick) < 0.05:
                self.speed_stick = 0
            if abs(self.turning_stick) < 0.05:
                self.turning_stick = 0
            speed = -self.speed_stick * 60
            turning_radius = -lookup(self.turning_stick *
                                     4, self.lookup_in, self.lookup_out)

            # Update car position, map, and LIDAR data
            self.update_plot(speed, turning_radius)

            self.send_tf_transformation()

            # Redraw the plots
            if i % 1 == 0:
                # Update the LIDAR plot
                self.plot_stuff()

                plt.draw()
                # Pause for 100ms to control the simulation update rate
                plt.pause(0.0001)

            self.append_training_data()

            if np.linalg.norm(np.array(self.goal) - np.array(self.position)) * self.resolution < 0.5:
                self.training_data.append(self.training_data_current_batch)
                self.training_data_current_batch = []
                self.position = list(random.choice(self.ok_position))
                self.starting_position = self.position
                self.angle = random.uniform(-np.pi, np.pi)
                self.send_tf_transformation()
                self.choose_goal()
                time.sleep(1)

            self.rate.sleep()

        # Keep the plot open
        plt.show()

    def choose_goal(self):
        self.goal = list(random.choice(self.ok_position))
        self.publish_coordinates(self.goal[0], self.goal[1])

    def append_training_data(self):
        data_entry = {
            "laser_angles": self.laser_angles.tolist(),
            "laser_ranges": self.laser_ranges.tolist(),
            "goal_angle": float(self.goal_angle),
            "goal_distance": float(self.goal_distance),
            "speed_stick": float(self.speed_stick),
            "turning_stick": float(self.turning_stick),
        }
        self.training_data_current_batch.append(data_entry)

    def handle_shutdown(self, signum, frame):
        """Save training data and clean up before exiting."""
        print("\nSaving training data before exiting...")
        self.save_training_data()
        print("Training data saved. Exiting gracefully.")
        exit(0)

    def save_training_data(self):
        """Save the training data to a file using pickle."""
        # Get the current script's directory
        current_directory = os.path.dirname(os.path.abspath(__file__))

        # Construct the full file path including the filename
        file_path = os.path.join(
            current_directory, "../training_data/training_data.pkl")

        # Log the resolved path for debugging
        rospy.logwarn(f"Saving training data to: {file_path}")

        # Save the training data using pickle
        with open(file_path, "wb") as file:
            pickle.dump(self.training_data, file)

        print(f"Training data saved to {file_path}")

    def publish_coordinates(self, x, y):
        point = Point()
        point.x = x
        point.y = y
        point.z = 0  # Assuming z = 0 for 2D map
        self.goal_publisher.publish(point)

    def update_plot(self, speed, turning_radius):
        """Update the car's position on the map."""

        self.car_motion_update(speed, turning_radius, delta_time=0.05)

        # Get the lidar scan using ray tracing
        self.generate_scan()

    def generate_scan(self):
        """Generate lidar scan using ray tracing."""
        self.laser_angles = np.linspace(-(self.LIDAR_ANGLE/2), (self.LIDAR_ANGLE/2),
                                        self.LIDAR_NUMBER_OF_POINTS)
        self.laser_angles = np.deg2rad(self.laser_angles)
        self.laser_ranges = np.zeros(self.LIDAR_NUMBER_OF_POINTS)
        max_range = 4 / self.resolution
        compensation_angle = 0
        for i, a in enumerate(self.laser_angles):
            # Calculate the direction vector based on the lidar angle
            direction = np.array(
                [np.cos(a + self.angle + compensation_angle), np.sin(a + self.angle + compensation_angle)])
            end_pos = self.position + max_range * direction
            max_pos = [self.map.shape[1]-1, self.map.shape[0]-1]
            min_pos = [0, 0]
            range = []

            if end_pos[0] > max_pos[0]:
                range.append(
                    np.abs((max_pos[0] - self.position[0])/direction[0]))

            if end_pos[0] < min_pos[0]:
                range.append(
                    np.abs((min_pos[0] - self.position[0])/direction[0]))

            if end_pos[1] > max_pos[1]:
                range.append(
                    np.abs((max_pos[1] - self.position[1])/direction[1]))

            if end_pos[1] < min_pos[1]:
                range.append(
                    np.abs((min_pos[1] - self.position[1])/direction[1]))

            if len(range) != 0:
                end_pos = self.position + min(range) * direction

            end_pos = np.ceil(end_pos)
            rounded_up_position = np.ceil(self.position)
            rounded_up_position = [
                int(rounded_up_position[0]), int(rounded_up_position[1])]
            end_pos = [int(end_pos[0]), int(end_pos[1])]

            self.laser_ranges[i] = self.resolution * \
                bresenham_raytrace(rounded_up_position, end_pos, self.map)

        self.scan.ranges = self.laser_ranges  # LIDAR range values

        self.scan_publisher.publish(self.scan)

    def car_motion_update(self, speed, turning_radius, delta_time):
        """Update the car's position and angle based on speed and turning radius."""

        turning_radius = turning_radius / self.resolution * -1

        if turning_radius == 0:
            # Straight line motion
            dx = speed * np.cos(self.angle)
            dy = speed * np.sin(self.angle)
            self.position = [self.position[0] + dx *
                             delta_time, self.position[1] + dy * delta_time]

        else:
            # Circular motion
            R = turning_radius
            # Calculate the angular velocity
            angular_velocity = speed / R
            # Update position and orientation
            new_angle = self.angle + angular_velocity * delta_time
            local_x = R * np.sin(angular_velocity * delta_time)
            local_y = R * (1 - np.cos(angular_velocity * delta_time))
            dx = np.cos(self.angle) * local_x - np.sin(self.angle) * local_y
            dy = np.sin(self.angle) * local_x + np.cos(self.angle) * local_y
            self.position = [self.position[0] + dx, self.position[1] + dy]
            self.angle = new_angle

        if self.angle > np.pi:
            self.angle -= 2*np.pi
        if self.angle < -np.pi:
            self.angle += 2*np.pi

    def send_tf_transformation(self, event=None):
        quaternion = tf.transformations.quaternion_from_euler(
            0, 0, self.angle * -1)
        self.tf_broadcaster.sendTransform(
            # Translation
            (self.position[0] * self.resolution,
                self.position[1] * self.resolution * -1, 0),  # different axes
            quaternion,
            rospy.Time.now(),
            "base_link",  # Child frame
            "map"  # Parent frame
        )

    def control_vector_callback(self, msg):
        self.control_vector = msg.data

    def angle_distance_callback(self, msg):
        self.goal_angle = msg.data[0]
        self.goal_distance = msg.data[1]

    def goal_callback(self, msg):
        self.goal[0] = msg.x
        self.goal[1] = msg.y
        self.training_data_current_batch = []

    def plot_stuff(self):
        """
        Plot the LIDAR scan on the second subplot and mark the goal position as a red star.

        Args:
            lidar_ax: The matplotlib axis to plot the LIDAR scan.
            laser_angles: Array of angles from the LIDAR.
            laser_ranges: Array of distances corresponding to the LIDAR angles.
            goal_angle: The angle (in radians) to the goal relative to the sensor.
            goal_distance: The distance to the goal.
        """
        import numpy as np  # Ensure NumPy is imported

        # Clear and set up the plot
        self.lidar_ax.clear()
        self.lidar_ax.set_title("LIDAR Scan")
        self.lidar_ax.set_xlim(-5, 5)
        self.lidar_ax.set_ylim(-5, 5)
        self.lidar_ax.set_xlabel("X")
        self.lidar_ax.set_ylabel("Y")

        # Convert LIDAR polar coordinates to Cartesian coordinates
        lidar_x = self.laser_ranges * np.sin(self.laser_angles)
        lidar_y = self.laser_ranges * np.cos(self.laser_angles)

        # Plot the LIDAR points
        self.lidar_ax.scatter(lidar_x, lidar_y, c='blue',
                              s=2, label="LIDAR Points")

        # Convert goal polar coordinates to Cartesian coordinates
        self.goal_angle = self.goal_angle * -1
        goal_x = self.goal_distance * np.sin(self.goal_angle)
        goal_y = self.goal_distance * np.cos(self.goal_angle)

        # Plot the goal as a red star
        self.lidar_ax.scatter(goal_x, goal_y, c='red',
                              marker='*', s=100, label="Goal")

        self.lidar_ax.plot([-0.2, 0.2, 0.2, -0.2, -0.2], [0, 0, -0.5, -0.5, 0], c='red',
                           marker='*', label="Car")

        # Add a legend for clarity
        self.lidar_ax.legend()


if __name__ == "__main__":
    # main()
    node = SimulationNode()
    rospy.spin()
