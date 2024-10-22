#!/usr/bin/env python3
import os
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import serial
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port

def talker(laser: hokuyo.Hokuyo):
    rospy.init_node('lidar_publisher')  # Initialize a ROS node
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)  # Publisher for /scan topic
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        data_dict = laser.get_single_scan()
        laser_data = np.array(list(data_dict.items()))
        laser_distances = laser_data[:,1]
        laser_distances = laser_distances/1000        #[m]
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "lidar_frame"
        
        # Populate LaserScan message fields
        scan.angle_min = np.deg2rad(-119.885)
        scan.angle_max = np.deg2rad(119.885)
        scan.angle_increment = np.deg2rad(119.885*2/681)
        scan.time_increment = 0.0667/682
        scan.scan_time = 0.1
        scan.range_min = 0.02
        scan.range_max = 4.0
        scan.ranges = laser_distances  # LIDAR range values
        # scan.ranges[(scan.ranges > scan.range_max) | (scan.ranges < scan.range_min)] = np.nan

        # Publish the LaserScan message
        pub.publish(scan)

        rate.sleep()

if __name__ == '__main__':
    uart_port_laser = '/dev/ttyACM0'
    uart_speed_laser = 19200
    draw_laser = 0

    laser_data = []
    laser_serial = serial.Serial(port=uart_port_laser, baudrate=uart_speed_laser, timeout=0.5)
    port_laser = serial_port.SerialPort(laser_serial)

    laser = hokuyo.Hokuyo(port_laser)

    laser.laser_off()
    laser_serial.reset_input_buffer()
    laser.laser_on() 
    try:
        talker(laser)
    except rospy.ROSInterruptException:
        pass