import serial
import time
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port
import sys
import signal
import os
import re
import threading
import multiprocessing


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


def new_file_name(os,re,directory):
    # List all files in the directory
    files = os.listdir(directory)

    # Filter files that match the desired pattern
    pattern = r'recorded_data_CAR_(\d+)\.npy'
    matching_files = [f for f in files if re.match(pattern, f)]

    # If there are no matching files, start numbering from 1
    if len(matching_files) == 0:
        highest_number = 0
    else:
        # Extract numbers from filenames and find the highest one
        numbers = [int(re.match(pattern, f).group(1)) for f in matching_files]
        highest_number = max(numbers)

    # Increment the highest number
    new_number = highest_number + 1
    return new_number

def stop_the_car(ser_ftdi,stop_data):
    for i in range(40):
        packed_data = prepare_data_to_PIC33(stop_data)
        ser_ftdi.write(packed_data)
        time.sleep(0.001)


def handle_interrupt(signal, frame):
    # Interrupt to break the loop with Ctrl+C and saving the data    
    global data_laser_saving
    global data
    global filename_LiDar
    global filename_CAR4

    # print(" pressed. Turning LiDar off and Exiting...")
    # laser.laser_off()   
    
    # # Save the data   
    # np.save(filename_LiDar, data_laser_saving)
    # np.save(filename_CAR4, data)
    # print("Data saved")

    # Add your code to handle the interrupt here
    sys.exit(0)

# Register the signal handler
signal.signal(signal.SIGINT, handle_interrupt)


count = 1200
data = np.zeros((count, 15), dtype=float)

########################################3
script_dir = os.path.dirname(os.path.abspath(__file__))
new_number = new_file_name(os,re,script_dir)

# Serial connection setting
ser_ftdi = setting_serial_PC_PIC()

# Calculate the delay in seconds for a frequency of 1000 Hz
delay = 1 / 1000  # 1 millisecond delay 


start_time = time.monotonic()


#ser_ftdi.reset_input_buffer()
time.sleep(0.001)
# Main function
if __name__=="__main__":
    ######################################################################

    print("START of driving")
    print("Press Ctrl+c to exit and save")
    # MAIN LOOP
    while 1:
        
        # Send data to CAR4        
        vector_to_send = [99, 0, 127, 127, 0]
 
     
        packed_data = prepare_data_to_PIC33(vector_to_send)
        ser_ftdi.write(packed_data)


        time.sleep(delay)

    #####################################################################