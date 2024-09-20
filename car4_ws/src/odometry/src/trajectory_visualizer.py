#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from odometry.msg import CarState

# Initialize the ROS node
rospy.init_node('trajectory_visualizer')

# Initialize variables to store the latest odometry data
latest_x = None
latest_y = None
x_coords = []
y_coords = []

# Callback function for receiving odometry messages
def callback(msg):
    global latest_x, latest_y
    # Store the latest odometry message
    latest_x = msg.x
    latest_y = msg.y

# Function to update the plot at a fixed rate
def update_plot(event):
    global latest_x, latest_y
    if latest_x is not None and latest_y is not None:
        # Append the latest data to the trajectory lists
        x_coords.append(latest_x)
        y_coords.append(latest_y)

        # Clear the previous plot and update it with the new trajectory
        plt.clf()  # Clear the previous plot
        plt.scatter(x_coords, y_coords, label='Trajectory Points', color='blue')
        plt.plot(x_coords, y_coords, linestyle='-', marker='o', color='red', label='Trajectory Path')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.axis('equal')
        plt.title('Real-time Trajectory Visualization')
        plt.legend()

        # Draw and pause to update the plot in real-time
        plt.draw()
        plt.pause(0.001)  # Small pause to allow for the plot to update

# Main function
def main():
    # Subscribe to the odometry topic
    rospy.Subscriber('odometry_data', CarState, callback)

    # Initialize the plot
    plt.ion()  # Turn on interactive mode for real-time updates

    # Set a timer to update the plot every 1 second
    rospy.Timer(rospy.Duration(0.2), update_plot)

    try:
        rospy.spin()  # Keep the node running until Ctrl+C is pressed
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
