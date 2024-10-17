#!/usr/bin/env python3
import rospy
import tf.transformations
import tf2_ros
from geometry_msgs.msg import Polygon
from nav_msgs.msg import Path
from tf2_geometry_msgs import PoseStamped
import threading
import serial
import time
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import signal
import os
import re
import threading
import multiprocessing
import tf

rospy.init_node("global_driver")

# Initialize global variables
path = None  # Global variable for the path (Polygon type)
position = None  # Global variable for the transformation from map to base_link
tf_listener = tf.TransformListener()
lookforward_distance = 2 #[m]
point_to_follow = None

def path_callback(msg):
    """
    Callback function for receiving the path.
    It updates the global 'path' variable.
    """
    global path
    path = []
    for point in msg.points:
        path.insert(0, point)
    rospy.loginfo("Received new path data.")

def update_tf():
    """
    Function to continuously listen to the transformation between 'map' and 'base_link'.
    Runs in a separate thread and updates the global 'tf_map_to_base' variable.
    """
    global lookforward_distance
    global position
    global point_to_follow
    global ser_ftdi
    rate = rospy.Rate(10)  # Update tf 10 times per second (10 Hz)
    
    while not rospy.is_shutdown():
        try:
            # Get the latest transform between 'map' and 'base_link'
            tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            (translation, quaternion) = tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            position = [translation[0], translation[1]]
            euler_angles = tf.transformations.euler_from_quaternion(quaternion)
            angle = euler_angles[2]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to lookup transformations")
        
        if path != None and position != None:
            min_distance = np.inf
            for point in path:
                distance = np.linalg.norm([point.x - position[0], point.y - position[1]])
                
                # Check if the point is beyond the lookforward distance
                if distance <= lookforward_distance:
                    point_to_follow = point
                    break
                else:
                    # Find the closest point
                    if distance < min_distance:
                        min_distance = distance
                        point_to_follow = point
        
        # vector_to_send = [99, 0, 254, 127, 0]
        # packed_data = prepare_data_to_PIC33(vector_to_send)
        # ser_ftdi.write(packed_data)
            

        if point_to_follow != None:
            turning_angle = np.arctan2(point_to_follow.y - position[1], point_to_follow.x - position[0]) - angle
            if turning_angle > np.pi:
                turning_angle = turning_angle - 2 * np.pi
            if turning_angle < -np.pi:
                turning_angle = turning_angle + 2 * np.pi
            # [control_data["PC"], control_data["MODE"],control_data["DIR"], control_data["SPEED"], control_data["MODESTEERING"]]
            if np.abs(turning_angle) <= 1.6:
                speed = 127+30
                dir = int(np.clip(127 - 200 * turning_angle, 1, 254))
            else:
                speed = 127-30
                if turning_angle > 0:
                    dir = 254
                else:
                    dir = 1
            vector_to_send = [99, 0, dir, speed, 0]
            rospy.logwarn("turning angle" + str(turning_angle))
            rospy.logwarn("car_angle" + str(angle))
            rospy.logwarn("point angle" + str(np.arctan2(point.y - position[1], point.x - position[0])))
            rospy.logwarn(vector_to_send)
            packed_data = prepare_data_to_PIC33(vector_to_send)
            ser_ftdi.write(packed_data)
            
            
        rate.sleep()

def setting_serial_PC_PIC():
    # Configure the serial port to SEND data
    port = '/dev/ttyUSB0'  # Change this to match your serial port on Ubuntu
    baud_rate = 115200  # Set your baudrate
    timeout = 1  # Set timeout value, if needed
    parity = serial.PARITY_ODD  # Set parity to odd
    bytesize = serial.EIGHTBITS  # Use 8 data bits
    stopbits = serial.STOPBITS_ONE  # Use 1 stop bit
    xonxoff = False  # Disable software flow control
    rtscts = False  # Disable hardware (RTS/CTS) flow control
    dsrdtr = False  # Disable hardware (DSR/DTR) flow control

    # Open the serial port
    ser_ftdi = serial.Serial(
        port=port,
        baudrate=baud_rate,
        parity=parity,
        bytesize=bytesize,
        stopbits=stopbits,
        timeout=timeout,
        xonxoff=xonxoff,
        rtscts=rtscts,
        dsrdtr=dsrdtr
    )
    return ser_ftdi

def prepare_data_to_PIC33(data_to_send):

    # Define the data to send   
    data_bytes = bytes(data_to_send)

    # Calculate CRC using XOR
    crc = 0

    for byte in data_bytes:
        crc ^= byte

    # Append CRC to the data
    data_with_crc = [16]+ data_to_send + [crc] + [126]

    # Pack the data into little-endian format
    packed_data = struct.pack('<' + 'B' * len(data_with_crc), *data_with_crc )

    return packed_data


# # Serial connection setting
# ser_ftdi = setting_serial_PC_PIC()

# # Calculate the delay in seconds for a frequency of 1000 Hz
# delay = 1 / 1000  # 1 millisecond delay 


# start_time = time.monotonic()


# #ser_ftdi.reset_input_buffer()
# time.sleep(0.001)
# # Main function

if __name__ == "__main__":
    try:

        ser_ftdi = setting_serial_PC_PIC()
        # Create a subscriber for the Polygon path topic
        rospy.Subscriber("/path", Polygon, path_callback)

        # Start a separate thread for updating the transform (tf)
        tf_thread = threading.Thread(target=update_tf)
        tf_thread.start()

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass