#!/usr/bin/env python3
import rospy
import tf
import serial
import struct
import numpy as np
from geometry_msgs.msg import Polygon, Point32
import threading
import signal
import sys
from std_msgs.msg import Float64MultiArray

class GlobalDriverNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("global_driver")

        # Set up instance variables
        self.path = []
        self.position = None
        self.lookforward_distance = 3  # [m]
        self.point_to_follow = None

        # Set up TF listener
        self.tf_listener = tf.TransformListener()

        # Set up subscriber and publisher
        self.path_subscriber = rospy.Subscriber("/path", Polygon, self.path_callback)
        self.point_publisher = rospy.Publisher("point_to_follow_global", Point32, queue_size=10)
        self.angle_distance_publisher = rospy.Publisher("point_to_follow_angle_distance", Float64MultiArray, queue_size=10)

        # Start a thread to continuously update transformations
        self.tf_thread = threading.Thread(target=self.update_tf)
        self.tf_thread.start()

    def path_callback(self, msg):
        """
        Callback function for receiving the path.
        It updates the instance's 'path' variable.
        """
        self.path = []
        for point in msg.points:
            self.path.insert(0, point)
        rospy.loginfo("Received new path data.")

    def update_tf(self):
        """
        Continuously listens to the transformation between 'map' and 'base_link'.
        Runs in a separate thread and updates the 'position' and 'point_to_follow' variables.
        """
        rate = rospy.Rate(10)  # Update tf 10 times per second (10 Hz)

        while not rospy.is_shutdown():
            try:
                # Get the latest transform between 'map' and 'base_link'
                self.tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
                (translation, quaternion) = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                self.position = [translation[0], translation[1]]
                euler_angles = tf.transformations.euler_from_quaternion(quaternion)
                angle = euler_angles[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to lookup transformations")
                continue

            # Update the point to follow
            if self.path and self.position:
                self.update_point_to_follow(angle)
                self.send_control_data(angle)
            rate.sleep()

    def update_point_to_follow(self, angle):
        min_distance = np.inf
        for point in self.path:
            distance = np.linalg.norm([point.x - self.position[0], point.y - self.position[1]])
            if distance <= self.lookforward_distance:
                self.point_to_follow = point
                break
            elif distance < min_distance:
                min_distance = distance
                self.point_to_follow = point

    def send_control_data(self, angle):
        if self.point_to_follow:
            
            turning_angle = np.arctan2(self.point_to_follow.y - self.position[1], self.point_to_follow.x - self.position[0]) - angle
            turning_angle = (turning_angle + np.pi) % (2 * np.pi) - np.pi

            self.point_publisher.publish(self.point_to_follow)
            distance = float(np.linalg.norm([self.point_to_follow.y - self.position[1], self.point_to_follow.x - self.position[0]]))
            msg = Float64MultiArray()
            msg.data = [turning_angle, distance]
            self.angle_distance_publisher.publish(msg)


if __name__ == "__main__":
    try:
        node = GlobalDriverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
