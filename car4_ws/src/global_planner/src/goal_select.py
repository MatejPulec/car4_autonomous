#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from PIL import Image
import os
import numpy as np

pub = None

def publish_coordinates(x, y):
    point = Point()
    point.x = x
    point.y = y
    point.z = 0  # Assuming z = 0 for 2D map
    pub.publish(point)

def onclick(event, ax, pgm_image):
    x, y = event.xdata, event.ydata
    if x is not None and y is not None:
        x = np.round(x)
        y = np.round(y)

        publish_coordinates(x, y)

        # Clear the plot and redraw the map
        ax.clear()
        ax.imshow(pgm_image, cmap='gray')
        
        # Mark the clicked point
        ax.plot(x, y, 'ro')
        plt.draw()

def main():
    global pub
    rospy.init_node('goal_coordinate_publisher', anonymous=True)
    pub = rospy.Publisher('/goal_coordinates', Point, queue_size=10)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    map_path = os.path.join(script_dir, "../map_mashup.pgm")
    pgm_image = Image.open(map_path)

    fig, ax = plt.subplots()
    ax.imshow(pgm_image, cmap='gray')

    cid = fig.canvas.mpl_connect('button_press_event', lambda event: onclick(event, ax, pgm_image))

    plt.show()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
