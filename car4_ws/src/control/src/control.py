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
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan

class ControlNode:
    def __init__(self):

        # Initialize ROS node
        rospy.init_node("control_node")

        # Set up serial connection
        self.ser_ftdi = self.setting_serial_PC_PIC()

        # Set up subscriber and publisher
        self.path_subscriber = rospy.Subscriber("/control_vector", Int32MultiArray, self.control_vector_callback)

        # Set up interrupt handling for clean exit
        signal.signal(signal.SIGINT, self.handle_interrupt)

    def control_vector_callback(self,msg):
        packed_data = self.prepare_data_to_PIC33(list(msg.data))
        self.ser_ftdi.write(packed_data)
        rospy.logwarn("here")


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
        self.ser_ftdi.write(packed_data)

        rospy.logwarn("Switched back to manual, terminating...")

        if self.ser_ftdi.is_open:
            self.ser_ftdi.close()

        sys.exit(0)

if __name__ == "__main__":
    try:
        node = ControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
