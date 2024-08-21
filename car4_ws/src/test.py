import numpy as np
import time

data = np.load('/home/mrmat420/car4_ws/src/car4_data/laser_data_4.npy')
data = data[150:, :, ]
np.save("laser_data_4_trimmed", data)
data = np.load('/home/mrmat420/car4_ws/src/car4_data/wheel_data_4.npy')
data = data[150:, :, ]
np.save("wheel_data_4_trimmed", data)
time.sleep(1)