#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from odometry.msg import CarState

# Initialize the ROS node
rospy.init_node('trajectory_visualizer')

# Initialize empty lists to store trajectory data
x_coords = []
y_coords = []

def callback(msg):
    # Extract relevant data from the message (e.g., x and y coordinates)
    x = msg.x
    y = msg.y

    # Append data to the lists
    x_coords.append(x)
    y_coords.append(y)

    # Update the plot
    plt.clf()  # Clear the previous plot
    plt.scatter(x_coords, y_coords, label='Trajectory Points', color='blue')
    plt.plot(x_coords, y_coords, linestyle='-', marker='o', color='red', label='Trajectory Path')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.axis('equal')
    plt.title('Real-time Trajectory Visualization')
    plt.legend()
    plt.pause(0.01)  # Pause to allow real-time updates

def main():
    # Subscribe to the topic (replace 'cmd_vel' with your actual topic name)
    rospy.Subscriber('odometry_data', CarState, callback)

    # Initialize the plot
    plt.ion()  # Turn on interactive mode for real-time updates

    try:
        rospy.spin()  # Keep the node running until Ctrl+C is pressed
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
