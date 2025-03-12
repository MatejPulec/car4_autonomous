#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Polygon
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from PIL import Image
import os
import threading
import tf

# Initialize ROS publishers and listener
pub = None
tf_listener = None

# Initialize plot elements
fig, ax = None, None
path_line = None
start_marker = None
end_marker = None
pos_line = None
pos_x, pos_y = [], []

# Variable to control whether to start drawing the path
start_drawing = False


def path_callback(msg):
    """Callback for handling path updates."""
    global path_line, start_marker, end_marker

    path_x = [point.x / 0.025 for point in msg.points]
    path_y = [point.y / 0.025 * -1 for point in msg.points]

    if path_x and path_y:
        start_marker.set_data(path_x[0], path_y[0])
        end_marker.set_data(path_x[-1], path_y[-1])
        path_line.set_data(path_x, path_y)

        plt.draw()
        plt.show(block=False)


def car4_wheels_data_callback(msg):
    """Callback for handling the car4_wheels_data and checking condition."""
    global start_drawing

    # Check if all elements at indices [3,4,5,6] are greater than 0.1
    if all(v > 0.1 for v in msg.data[3:7]):
        start_drawing = True


def update_pos():
    """Continuously updates the position using the latest available TF data."""
    global pos_x, pos_y, pos_line, start_drawing

    rospy.sleep(1)  # Give time for TF buffer to populate

    while not rospy.is_shutdown():
        if start_drawing:  # Only start drawing the trajectory if the condition is met
            try:
                # Get the latest time available from the TF buffer
                latest_time = tf_listener.getLatestCommonTime(
                    "map", "AMCL_for_visualization")

                # Wait for the transform to be available at the latest time
                tf_listener.waitForTransform(
                    "map", "AMCL_for_visualization", latest_time, rospy.Duration(10.0))

                # Get the transform at the most recent available time (ignoring exact timestamps)
                (translation, _) = tf_listener.lookupTransform(
                    "map", "AMCL_for_visualization", latest_time)

                rospy.logwarn(f"New Position: {latest_time}")

                # Convert and scale position
                x = translation[0] / 0.025
                y = translation[1] / 0.025 * -1

                pos_x.append(x)
                pos_y.append(y)
                pos_line.set_data(pos_x, pos_y)

                plt.draw()
                plt.show(block=False)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"TF lookup failed: {e}, retrying...")

        rospy.sleep(0.1)  # Small delay to prevent busy-waiting


def main():
    global pub, tf_listener
    global fig, ax, path_line, start_marker, end_marker, pos_line

    rospy.init_node('path_visualizer', anonymous=True)
    pub = rospy.Publisher('/goal_coordinates', Point, queue_size=10)
    tf_listener = tf.TransformListener()

    rospy.Subscriber('/path', Polygon, path_callback)
    # Subscribe to the car4_wheels_data topic
    rospy.Subscriber('/car4_wheels_data', Float32MultiArray,
                     car4_wheels_data_callback)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_path = os.path.join(script_dir, "../map_mashup.pgm")
    pgm_image = Image.open(map_path)

    fig, ax = plt.subplots()
    ax.imshow(pgm_image, cmap='gray')

    start_marker, = ax.plot([], [], 'go', markersize=8)
    end_marker, = ax.plot([], [], 'ro', markersize=8)
    pos_line, = ax.plot([], [], 'b-', linewidth=2)
    path_line, = ax.plot([], [], color='green',
                         linestyle='dotted', linewidth=2)

    pos_thread = threading.Thread(target=update_pos)
    pos_thread.daemon = True
    pos_thread.start()

    plt.show()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
