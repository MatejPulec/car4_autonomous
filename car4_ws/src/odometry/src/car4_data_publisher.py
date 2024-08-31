#!/usr/bin/env python3
import os
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg

def talker(data):
    rospy.init_node('car4_wheels_publisher')
    hz = rospy.get_param('~hz', 10)
    pub = rospy.Publisher('car4_wheels_data', numpy_msg(Float32MultiArray), queue_size=10)
    rate = rospy.Rate(hz)
    data_idx = 0
    data_length = data.shape[0]
    while not rospy.is_shutdown():
        msg = Float32MultiArray()
        msg.layout.data_offset = 0
        msg.data = data[data_idx, 0, :].tolist()
        pub.publish(msg)
        data_idx += 1
        if data_idx >= data_length:
            break
        rate.sleep()


if __name__ == '__main__':
    try:
        # Retrieve the 'hz' parameter from the parameter server
        script_dir = os.path.dirname(os.path.realpath(__file__))
        default_file_path = os.path.join(script_dir, '../../car4_data/wheel_data_4_trimmed.npy')
        file_path = rospy.get_param('/wheel_data_publisher/wheel_data_file', default_file_path)
        data = np.load(file_path)
        talker(data)
    except rospy.ROSInterruptException:
        pass
