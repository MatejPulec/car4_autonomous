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
from sensor_msgs.msg import LaserScan

class LocalDriverNode:
    def __init__(self):

        self.range_threshold = 0.7
        self.repulsive_coef = 1
        self.speed = 160
        # Initialize ROS node
        rospy.init_node("local_driver")

        # Set up serial connection
        self.ser_ftdi = self.setting_serial_PC_PIC()

        # Set up subscriber and publisher
        self.path_subscriber = rospy.Subscriber("/point_to_follow_angle_distance", Float64MultiArray, self.local_planner_callback)
        self.path_subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_scann_callback)

        # Set up interrupt handling for clean exit
        signal.signal(signal.SIGINT, self.handle_interrupt)

    def angle_distance_callback(self, msg):

        self.goal_angle = msg[0]
        self.goal_distance = msg[1]

    def laser_scann_callback(self,msg):

        attractive_force_vector = 100 * [np.sin(self.goal_angle), np.cos(self.goal_angle)]
        repulsive_force_vector = [0,0]

        if not hasattr(self, 'laser_angles'):
            self.laser_angles = np.arange(msg.angle_max, msg.angle_min -
                                     msg.angle_increment, -msg.angle_increment)
            
        for range, laser_angle in zip(msg.ranges, self.laser_angles):
            if range < self.range_threshold:
                # diff of 1/2 n (1/x - 1/t)**2
                repulsive_intensity = self.repulsive_coef*(self.range_threshold-range)/(self.range_threshold*range**3)
                repulsive_force_vector = repulsive_force_vector + repulsive_intensity * [np.sin(laser_angle), np.cos(laser_angle)]
        
        total_vector = attractive_force_vector - repulsive_force_vector
        total_angle = np.arctan(total_vector[0]/total_vector[1])

    
    # # No use of lidar
    # def laser_scann_callback(self,msg):

    #     if abs(self.goal_angle) <= 1.6:
    #         speed = 127 + 30
    #         dir = int(np.clip(127 - 200 * self.goal_angle, 1, 254))
    #     else:
    #         speed = 127 - 30
    #         dir = 254 if self.goal_angle > 0 else 1
    #     return speed, dir


    def send_control_data(self, angle):
        if self.point_to_follow:
            
            turning_angle = np.arctan2(self.point_to_follow.y - self.position[1], self.point_to_follow.x - self.position[0]) - angle
            turning_angle = (turning_angle + np.pi) % (2 * np.pi) - np.pi

            speed, dir = self.calculate_speed_direction(turning_angle)
            vector_to_send = [99, 0, dir, speed, 0]

            self.point_publisher.publish(self.point_to_follow)
            distance = float(np.linalg.norm([self.point_to_follow.y - self.position[1], self.point_to_follow.x - self.position[0]]))
            msg = Float64MultiArray()
            msg.data = [turning_angle, distance]
            self.angle_distance_publisher.publish(msg)

        else:
            vector_to_send = [0, 0, 127, 127, 0]

        # packed_data = self.prepare_data_to_PIC33(vector_to_send)
        # self.ser_ftdi.write(packed_data)

    def calculate_speed_direction(self, turning_angle):
        if abs(turning_angle) <= 1.6:
            speed = 127 + 30
            dir = int(np.clip(127 - 200 * turning_angle, 1, 254))
        else:
            speed = 127 - 30
            dir = 254 if turning_angle > 0 else 1
        return speed, dir

    def setting_serial_PC_PIC(self):
        # Configure the serial port to SEND data
        port = '/dev/ttyUSB0'
        baud_rate = 115200
        ser_ftdi = serial.Serial(
            port=port,
            baudrate=baud_rate,
            parity=serial.PARITY_ODD,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        return ser_ftdi

    def prepare_data_to_PIC33(self, data_to_send):
        data_bytes = bytes(data_to_send)
        crc = 0
        for byte in data_bytes:
            crc ^= byte
        data_with_crc = [16] + data_to_send + [crc] + [126]
        return struct.pack('<' + 'B' * len(data_with_crc), *data_with_crc)

    def handle_interrupt(self, sig, frame):
        # Send stop command to the PIC33
        vector_to_send = [0, 0, 127, 127, 0]
        packed_data = self.prepare_data_to_PIC33(vector_to_send)
        # self.ser_ftdi.write(packed_data)

        rospy.logwarn("Switched back to manual, terminating...")

        # if self.ser_ftdi.is_open:
        #     self.ser_ftdi.close()

        sys.exit(0)

if __name__ == "__main__":
    try:
        node = LocalDriverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
