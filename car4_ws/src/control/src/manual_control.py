#!/usr/bin/env python3
import rospy
import pygame
from std_msgs.msg import Int32MultiArray

class KeyControlNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("key_control_driver")

        # Set up the publisher for control vectors
        self.control_vector_publisher = rospy.Publisher("/control_vector", Int32MultiArray, queue_size=10)

        # Initialize pygame for keyboard input
        pygame.init()

        # Set the window size (it's invisible, just needed for pygame to work)
        pygame.display.set_mode((1, 1))

    def send_control_vector(self, control_vector):
        msg = Int32MultiArray()
        msg.data = control_vector
        self.control_vector_publisher.publish(msg)

    def run(self):
        print("Press the arrow keys to control the robot:")
        print("Up Arrow: Forward, Left Arrow: Left, Right Arrow: Right, Down Arrow: Backward")
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown("Shutting down key control node.")
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        # Forward
                        control_vector = [99, 0, 127, 200, 0]    
                        self.send_control_vector(control_vector)
                    elif event.key == pygame.K_LEFT:
                        # Left
                        control_vector = [99, 0, 2, 200, 0]
                        self.send_control_vector(control_vector)
                    elif event.key == pygame.K_RIGHT:
                        # Right
                        control_vector = [99, 0, 253, 200, 0]
                        self.send_control_vector(control_vector)
                    elif event.key == pygame.K_DOWN:
                        # Backward
                        control_vector = [99, 0, 127, 100, 0]
                        self.send_control_vector(control_vector)

if __name__ == "__main__":
    try:
        node = KeyControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
