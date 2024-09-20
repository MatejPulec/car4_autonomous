#!/usr/bin/env python3

import serial
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
import rospy
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg

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

def clear_serial_buffer(serial_port):
    # Read and discard any existing data in the serial buffer
    serial_port.reset_input_buffer()

def read_message_from_CAR4(serial_port, timeout):
    data = [float('nan')] * 14 # allocate space

    start_time = time.time()  # save time

    bytes_to_read = serial_port.in_waiting

    
    while True:
        end_reading = False  # whether to exit this loop

        # Find the starting byte in message from UART
        while True:
            # Read start byte
            start_byte = serial_port.read(1)
            if not start_byte:
                end_reading = True
                break

            start_byte = int.from_bytes(start_byte, 'little')

            # Check for timeout
            if time.time() - start_time >= timeout:
                end_reading = True
                break

            # Check whether the byte is the start byte
            if start_byte == 142:
                break

        # Check whether to exit this loop based on the previous decision
        if end_reading:
            break

        # Read all
        data[0:14] = struct.unpack('<14f', serial_port.read(56))

        # Check for timeout
        if time.time() - start_time >= timeout:
            break

        # Read end byte
        end_byte = serial_port.read(1)

        if not end_byte:
            break

        end_byte = int.from_bytes(end_byte, 'little')

        # Check whether the last byte is the end byte / if not throw away
        if end_byte == 98:            
            break     

    return data


def read_message(serial_port, timeout):
    data = [float('nan')] * 15  # allocate space

    start_time = time.time()  # save time

    bytes_to_read = serial_port.in_waiting
    print(bytes_to_read)
    if bytes_to_read > 60:
        serial_port.read(bytes_to_read-60)
    print(serial_port.in_waiting)
    
    while True:
        end_reading = False  # whether to exit this loop

        # find the starting byte
        while True:
            # read start byte
            start_byte = serial_port.read(1)
            if not start_byte:
                end_reading = True
                break

            start_byte = int.from_bytes(start_byte, 'little')

            # check for timeout
            if time.time() - start_time >= timeout:
                end_reading = True
                break

            # check whether the byte is the start byte
            if start_byte == 142:
                break

        # check whether to exit this loop based on the previous decision
        if end_reading:
            break

        # read first two singles
        data[0:2] = struct.unpack('<ff', serial_port.read(8))

        # check for timeout
        if time.time() - start_time >= timeout:
            break

        # read encoder error
        encoder_error_bytes = serial_port.read(2)
        if not encoder_error_bytes:
            break
        data[2] = struct.unpack('<H', encoder_error_bytes)[0]

        # check for timeout
        if time.time() - start_time >= timeout:
            break

        # read the rest of the data
        data[3:] = struct.unpack('<12f', serial_port.read(48))

        # check for timeout
        if time.time() - start_time >= timeout:
            break

        # read end byte
        end_byte = serial_port.read(1)
        if not end_byte:
            break
        end_byte = int.from_bytes(end_byte, 'little')

        # check whether the last byte is the end byte
        if end_byte == 98:
            break

    return data


def talker():
    rospy.init_node('car4_wheels_publisher')
    hz = 100
    pub = rospy.Publisher('car4_wheels_data', numpy_msg(Float32MultiArray), queue_size=10)
    rate = rospy.Rate(hz)
    port = setting_serial_PC_PIC()
    clear_serial_buffer(port)
    data = np.zeros((15), dtype=float)
    start_time = time.time()
    while not rospy.is_shutdown():
        msg = Float32MultiArray()
        data[0] = time.time() - start_time
        data[1:] = read_message_from_CAR4(port, 1)
        msg.layout.data_offset = 0
        msg.data = data.tolist()
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass