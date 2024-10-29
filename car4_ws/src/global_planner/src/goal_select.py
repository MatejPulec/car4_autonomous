#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Polygon
import matplotlib.pyplot as plt
from PIL import Image
import os
import numpy as np
import time
import threading  # For running the timed function in a background thread
import tf

# tf_listener = tf.TransformListener()

pub = None
drawing = 0

def publish_coordinates(x, y):
    point = Point()
    point.x = x
    point.y = y
    point.z = 0  # Assuming z = 0 for 2D map
    pub.publish(point)

def path_callback(msg):
    path_x = []
    path_y = []

    time.sleep(0.001)

    # Extract the coordinates from the Polygon message
    path_x = [point.x/0.025 for point in msg.points]
    path_y = [point.y/0.025*-1 for point in msg.points]

    if path_x and path_y:
        # Plot the updated path
        trajectory.set_data(path_x, path_y)
        ax.draw_artist(trajectory)
        plt.draw()
        plt.show(block=False)

def onclick(event, ax):
    x, y = event.xdata, event.ydata
    if x is not None and y is not None:
        x = np.round(x)
        y = np.round(y)

        publish_coordinates(x, y)

        goal.set_data(x, y)
        ax.draw_artist(goal)
        
        plt.draw()
        plt.show(block=False)

def update_pos():
    while not rospy.is_shutdown():
        time.sleep(1)  # Run this function every second
        tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(5.0))
        (translation, quaternion) = tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        x = translation[0]/0.025
        y = translation[1]/0.025 * -1
        if x is not None and y is not None:
            x = np.round(x)
            y = np.round(y)

            pos.set_data(x, y)
            ax.draw_artist(pos)

            plt.draw()
            plt.show(block=False)

def main():
    global pub
    global goal
    global pos
    global trajectory
    global ax
    global tf_listener

    rospy.init_node('goal_coordinate_publisher', anonymous=True)
    pub = rospy.Publisher('/goal_coordinates', Point, queue_size=10)
    tf_listener = tf.TransformListener()

    # Subscribe to the path topic
    rospy.Subscriber('/path', Polygon, path_callback)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_path = os.path.join(script_dir, "../map_mashup.pgm")
    pgm_image = Image.open(map_path)

    fig, ax = plt.subplots()
    ax.imshow(pgm_image, cmap='gray')
    goal, = ax.plot([], [], 'ro')
    pos, = ax.plot([], [], 'go')
    trajectory, = ax.plot([], [], 'bo')

    # Capture mouse clicks to send goal coordinates
    cid = fig.canvas.mpl_connect('button_press_event', lambda event: onclick(event, ax))

    # Start the update_pos function in a background thread
    pos_thread = threading.Thread(target=update_pos)
    pos_thread.daemon = True  # Make sure the thread exits when the main program exits
    pos_thread.start()

    # Spin in a separate thread while keeping the plot interactive
    plt.show()  # Non-blocking to allow ROS callbacks to work
    plt.pause(0.001)
    rospy.spin()  # Keep the node running to receive messages

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
