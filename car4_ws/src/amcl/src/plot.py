import rospy
import tf
import os
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import signal
import sys

# Initialize ROS node
rospy.init_node('map_to_base_link_plotter', anonymous=True)

# Initialize TF listener
tf_listener = tf.TransformListener()

# Load the map image
script_dir = os.path.dirname(os.path.realpath(__file__))
file_path = os.path.join(script_dir, '../map_mashup.pgm')

# Load map as numpy array
with Image.open(file_path) as img:
    map_img = np.array(img)

# List to store positions
positions = []

def plot_positions_on_map():
    plt.clf()  # Clear the plot
    plt.imshow(map_img, cmap='gray')  # Assuming the map is grayscale
    if positions:  # Only plot if there are positions
        x_positions, y_positions = zip(*positions)  # Unzip the list of positions
        # Convert to pixel coordinates and invert the Y-axis
        plt.plot(np.array(x_positions) / 0.025, -np.array(y_positions) / 0.025, 'ro')  # Plot positions as red dots
    plt.title('Robot Positions (map -> base_link)')
    plt.show(block=True)  # Show the plot and block execution until the window is closed


# Signal handler for termination
def signal_handler(sig, frame):
    print("\nTerminating the script. Plotting the robot's path...")
    plot_positions_on_map()  # Plot all positions when terminating
    sys.exit(0)  # Exit the script

# Main loop to continuously get the transformation and store the position
def plot_map_to_base_link():
    rate = rospy.Rate(10.0)  # 10 Hz
    plt.ion()  # Interactive mode for live plot updates

    while not rospy.is_shutdown():
        try:
            # Get the transformation from 'map' to 'odom'
            tf_listener.waitForTransform("map", "odom", rospy.Time(0), rospy.Duration(1.0))
            (map_to_odom_translation, map_to_odom_quaternion) = tf_listener.lookupTransform("map", "odom", rospy.Time(0))

            # Get the transformation from 'odom' to 'base_link'
            tf_listener.waitForTransform("odom", "base_link", rospy.Time(0), rospy.Duration(1.0))
            (odom_to_base_translation, odom_to_base_quaternion) = tf_listener.lookupTransform("odom", "base_link", rospy.Time(0))

            tf_listener.waitForTransform("map", "AMCL", rospy.Time(0), rospy.Duration(1.0))
            (map_to_amcl_translation, map_to_amcl_quaternion) = tf_listener.lookupTransform("map", "AMCL", rospy.Time(0))

            # Combine transformations to get 'map' to 'base_link'
            x = map_to_odom_translation[0] + odom_to_base_translation[0]
            y = map_to_odom_translation[1] + odom_to_base_translation[1]

            roll, pitch, yaw = tf.transformations.euler_from_quaternion(map_to_amcl_quaternion)
            print(map_to_amcl_translation, yaw)

            # Append the current position to the list
            positions.append((x, y))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to lookup transformations")

        rate.sleep()

# Start the plotting process and set up signal handling
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)  # Catch the Ctrl+C signal
    plot_map_to_base_link()
