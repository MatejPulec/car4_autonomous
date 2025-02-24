#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Polygon, PoseStamped, Quaternion
import matplotlib.pyplot as plt
from PIL import Image
import os
import numpy as np
import time
import threading  # For running the timed function in a background thread
import tf
import tf.transformations as tf_trans

# tf_listener = tf.TransformListener()

pub = None
drawing = 0


def publish_coordinates(x, y):
    point = Point()
    point.x = x
    point.y = y
    point.z = 0  # Assuming z = 0 for 2D map
    pub.publish(point)


def publish_reset_position(x, y, angle):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0  # Assuming z = 0 for 2D map
    quaternion = tf_trans.quaternion_from_euler(0, 0, angle)
    msg.pose.orientation = Quaternion(*quaternion)
    reset_pos_pub.publish(msg)


def path_callback(msg):
    path_x = []
    path_y = []

    time.sleep(0.0001)

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
    global reset_position
    x, y = event.xdata, event.ydata
    if x is not None and y is not None:
        x = np.round(x)
        y = np.round(y)

        if event.button == 1:  # Left-click: Set goal
            publish_coordinates(x, y)
            goal.set_data(x, y)
            ax.draw_artist(goal)
        elif event.button == 3:  # Right-click: Send reset position message
            if reset_position != 0:
                dx = x-reset_position[0]
                dy = y-reset_position[1]
                angle = np.arctan2(dy, dx)
                publish_reset_position(
                    reset_position[0], reset_position[1], angle)
                reset_position = 0
            else:
                reset_position = [x, y]

        plt.draw()
        plt.show(block=False)


def update_pos():
    while not rospy.is_shutdown():
        time.sleep(1)  # Run this function every two seconds
        tf_listener.waitForTransform(
            "map", "base_link", rospy.Time(0), rospy.Duration(10.0))
        (translation, quaternion) = tf_listener.lookupTransform(
            "map", "base_link", rospy.Time(0))
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
    global reset_pos_pub
    global reset_position

    rospy.init_node('goal_coordinate_publisher', anonymous=True)
    pub = rospy.Publisher('/goal_coordinates', Point, queue_size=10)
    reset_pos_pub = rospy.Publisher(
        '/position_reset', PoseStamped, queue_size=10)
    tf_listener = tf.TransformListener()

    # Subscribe to the path topic
    rospy.Subscriber('/path', Polygon, path_callback)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_path = os.path.join(script_dir, "../map_mashup.pgm")
    pgm_image = Image.open(map_path)

    fig, ax = plt.subplots()
    ax.imshow(pgm_image, cmap='gray')
    goal, = ax.plot([], [], 'ro', zorder=9)
    pos, = ax.plot([], [], 'go', zorder=10)
    trajectory, = ax.plot([], [], 'bo')

    reset_position = 0

    # Capture mouse clicks to send goal coordinates
    cid = fig.canvas.mpl_connect(
        'button_press_event', lambda event: onclick(event, ax))

    # Start the update_pos function in a background thread
    pos_thread = threading.Thread(target=update_pos)
    pos_thread.daemon = True  # Make sure the thread exits when the main program exits
    pos_thread.start()

    # Spin in a separate thread while keeping the plot interactive
    plt.show()  # Non-blocking to allow ROS callbacks to work
    plt.pause(0.0001)
    rospy.spin()  # Keep the node running to receive messages


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
