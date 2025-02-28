#!/usr/bin/env python3
import rospy
import pygame
from std_msgs.msg import Int32MultiArray
import sys

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

        # Control vector when no key is pressed
        self.default_vector = [99, 0, 127, 127, 0]

    def send_control_vector(self, control_vector):
        msg = Int32MultiArray()
        msg.data = control_vector
        self.control_vector_publisher.publish(msg)

    def run(self):
        print("Press the arrow keys to control the robot:")
        print("Up Arrow: Forward, Left Arrow: Left, Right Arrow: Right, Down Arrow: Backward")

        # Set the rate for 100 Hz
        rate = rospy.Rate(100)
        
        try:
            while not rospy.is_shutdown():
                # Process pygame events to prevent it from freezing
                pygame.event.pump()

                # Check the state of all keys
                keys = pygame.key.get_pressed()
                if keys[pygame.K_LSHIFT]:
                    speed = 80
                else:
                    speed = 50


                if keys[pygame.K_UP]:
                    if keys[pygame.K_LEFT]:
                        control_vector = [99, 0, 2, 127 + speed, 0]
                    elif keys[pygame.K_RIGHT]:
                        control_vector = [99, 0, 253, 127 + speed, 0]
                    else:
                        control_vector = [99, 0, 127, 127 + speed, 0]  # Forward    
                elif keys[pygame.K_DOWN]:
                    if keys[pygame.K_LEFT]:
                        control_vector = [99, 0, 2, 127 - speed/1.85, 0]
                    elif keys[pygame.K_RIGHT]:
                        control_vector = [99, 0, 253, 127 - speed/1.85, 0]
                    else:
                        control_vector = [99, 0, 127, 127 - speed/1.85, 0]
                else:
                    control_vector = self.default_vector   # Default when no key is pressed

                self.send_control_vector(control_vector)

                # Sleep to maintain the loop frequency at 100 Hz
                rate.sleep()
        
        except rospy.ROSInterruptException:
            # Handle ROS shutdown signal
            pass
        except KeyboardInterrupt:
            # Send final control vector before exiting on CTRL+C
            self.send_control_vector([0, 0, 127, 127, 0])
            print("\nExiting...")
        finally:
            pygame.quit()
            sys.exit(0)

if __name__ == "__main__":
    node = KeyControlNode()
    node.run()
