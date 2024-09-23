#!/usr/bin/env python3
import os
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

def talker(data):
    rospy.init_node('lidar_publisher')  # Initialize a ROS node
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)  # Publisher for /scan topic
    rate = rospy.Rate(10) # 10hz
    data_idx = 0
    data_length = data.shape[0]
    data = data/1000        #[m]
    while not rospy.is_shutdown():
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
        scan.ranges = data[data_idx, :, 1]  # LIDAR range values
        # scan.ranges[(scan.ranges > scan.range_max) | (scan.ranges < scan.range_min)] = np.nan

        # Publish the LaserScan message
        pub.publish(scan)
        data_idx += 1
        if data_idx >= data_length:
            break
        rate.sleep()

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.realpath(__file__))
    default_file_path = os.path.join(script_dir, '../../car4_data/laser_data_test_3.npy')
    file_path = rospy.get_param('/laser_data_publisher/laser_data_file', default_file_path)
    data = np.load(file_path)
    try:
        talker(data)
    except rospy.ROSInterruptException:
        pass