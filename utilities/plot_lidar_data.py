import serial
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port
import sys
import signal
import functools


uart_port = 'COM9'
uart_speed = 19200


def handle_interrupt(signal, frame):
    global distances, angles

    print("Ctrl+C pressed. Turning LiDar off and Exiting...")
    laser.laser_off()


    data = np.column_stack((angles, distances))

    print(data)

    # Save the data to a CSV file
    np.savetxt('garbage.csv', data, delimiter=',', header='Angle, Distance', fmt='%.2f', comments='')

    # Add your code to handle the interrupt here
    sys.exit(0)

# Register the signal handler
signal.signal(signal.SIGINT, handle_interrupt)


if __name__ == '__main__':
    laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
    port = serial_port.SerialPort(laser_serial)

    laser = hokuyo.Hokuyo(port)

    laser.laser_on()

    fig, ax = plt.subplots()
    line, = ax.plot(0, 0)
    ax.set_xlim([-4000, 4000])
    ax.set_ylim([-4000, 4000])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Example Figure')


    while 1:
        data_dict = laser.get_single_scan()
        
        data = np.array(list(data_dict.items()))
        angles = np.deg2rad(data[:,0])
        distances = data[:,1]

        Y = distances * np.cos(angles)
        X = distances * np.sin(angles)

        line.set_ydata(Y)
        line.set_xdata(X)
        fig.canvas.draw()
        plt.pause(0.05)

    laser.laser_off()