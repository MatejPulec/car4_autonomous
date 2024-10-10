#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Polygon
import matplotlib.pyplot as plt
from PIL import Image
import os
import numpy as np
import time

pub = None
drawing = 0
# path_x, path_y = [], []  # Store the path coordinates

def publish_coordinates(x, y):
    point = Point()
    point.x = x
    point.y = y
    point.z = 0  # Assuming z = 0 for 2D map
    pub.publish(point)

def path_callback(msg):

    path_x = []
    path_y = []

    time.sleep(0.5)

    # Log how many points we received
    rospy.logwarn(f"Received path with {len(msg.points)} points.")

    # Extract the coordinates from the Polygon message
    path_x = [point.x for point in msg.points]
    path_y = [point.y for point in msg.points]

    # Log the extracted coordinates
    rospy.logwarn(f"Extracted coordinates: {list(zip(path_x, path_y))}")

    if path_x and path_y:
        # Plot the updated path
        plt.gca().plot(path_x, path_y, marker='o', linestyle='-', color='b')
        plt.draw()
        rospy.loginfo("Path has been plotted.")
    else:
        rospy.logwarn("No points to plot.")

def onclick(event, ax, pgm_image):
    x, y = event.xdata, event.ydata
    if x is not None and y is not None:
        x = np.round(x)
        y = np.round(y)

        publish_coordinates(x, y)

        # Clear the plot and redraw the map
        ax.clear()
        ax.imshow(pgm_image, cmap='gray')

        # Mark the clicked point
        ax.plot(x, y, 'ro')

        # Redraw the path, if available
        # if path_x and path_y:
        #     ax.plot(path_x, path_y, marker='o', linestyle='-', color='b')
        
        plt.draw()
        plt.show(block=False)

def main():
    global pub

    rospy.init_node('goal_coordinate_publisher', anonymous=True)
    pub = rospy.Publisher('/goal_coordinates', Point, queue_size=10)

    # Subscribe to the path topic
    rospy.Subscriber('/path', Polygon, path_callback)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_path = os.path.join(script_dir, "../map_mashup.pgm")
    pgm_image = Image.open(map_path)

    fig, ax = plt.subplots()
    ax.imshow(pgm_image, cmap='gray')

    # Capture mouse clicks to send goal coordinates
    cid = fig.canvas.mpl_connect('button_press_event', lambda event: onclick(event, ax, pgm_image))

    # Spin in a separate thread while keeping the plot interactive
    plt.show()  # Non-blocking to allow ROS callbacks to work
    plt.pause(0.001)
    rospy.spin()  # Keep the node running to receive messages

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
