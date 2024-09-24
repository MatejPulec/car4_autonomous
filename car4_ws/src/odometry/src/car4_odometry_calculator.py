#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from rospy.numpy_msg import numpy_msg
from odometry.msg import CarState
import tf
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

def inverseAckermannAngles(angles, wheeltrack_w, wheelbase_L):
    """
    Description:
        Calculates radii of the vehicle and front and rear angles
    Parameters:
        angles: four element vector with wheel angles in this order: delta_FL, delta_FR, delta_RL, delta_RR
        wheeltrack_w: wheeltrack of the vehicle
        wheelbase_L: wheelbase of the vehicle
    Returns:
        Calculated radii and angles in this order: R_F, R_R, R_FL, R_FR, R_RL, R_RR, delta_F, delta_R, R, R0, beta
    """

    # use variables with meaningful names
    # the car reports angles with inverted signs, so a correction is needed
    delta_FL = -angles[0]
    delta_FR = -angles[1]
    delta_RL = angles[2]
    delta_RR = angles[3]

    # calculate radii and angles of the front axle
    if delta_FL == delta_FR:  # if angles are the same, radii are Inf
        R_FL = np.inf
        R_FR = np.inf
        R_F = np.inf
        delta_F = delta_FL  # use one of the angles
    else:  # if angles are not the same, calculate values normally
        R_FL = wheeltrack_w / np.sin(delta_FL - delta_FR) * np.sin(delta_FR)
        R_FR = wheeltrack_w / np.sin(delta_FL - delta_FR) * np.sin(np.pi - delta_FL)
        R_F = np.sign(R_FL) * np.sqrt(R_FL**2 + (wheeltrack_w / 2)**2 - 2 * R_FL * (wheeltrack_w / 2) * np.cos(np.pi - delta_FL))
        # different only on 7th decimal place:
        # R_F_v2 = np.sign(R_FR) * np.sqrt(R_FR**2 + (wheeltrack_w / 2)**2 - 2 * R_FR * (wheeltrack_w / 2) * np.cos(delta_FR))
        delta_F = delta_FL - np.arcsin(np.sin(np.pi - delta_FL) * wheeltrack_w / (2 * R_F))

    # calculate radii and angles of the rear axle
    if delta_RL == delta_RR:  # if angles are the same, radii are Inf
        R_RL = np.inf
        R_RR = np.inf
        R_R = np.inf
        delta_R = delta_RL  # use one of the angles
    else:  # if angles are not the same, calculate values normally
        R_RL = wheeltrack_w / np.sin(delta_RL - delta_RR) * np.sin(delta_RR)
        R_RR = wheeltrack_w / np.sin(delta_RL - delta_RR) * np.sin(np.pi - delta_RL)
        R_R = np.sign(R_RL) * np.sqrt(R_RL**2 + (wheeltrack_w / 2)**2 - 2 * R_RL * (wheeltrack_w / 2) * np.cos(np.pi - delta_RL))
        delta_R = delta_RL - np.arcsin(np.sin(np.pi - delta_RL) * wheeltrack_w / (2 * R_R))

    # calculate radii and angles of the center point
    if R_R != np.inf and R_R != -np.inf:
        R = np.cos(delta_R) * R_R
        R0 = np.sqrt(R_R**2 + (wheelbase_L / 2)**2 - 2 * R_R * wheelbase_L / 2 * np.cos(np.pi / 2 - delta_R))
        a = np.sin(delta_R) * R_R
    elif R_F != np.inf and R_F != -np.inf:
        R = np.cos(delta_F) * R_F
        R0 = np.sqrt(R_F**2 + (wheelbase_L / 2)**2 - 2 * R_F * wheelbase_L / 2 * np.cos(np.pi / 2 - delta_F))
        a = wheelbase_L - np.sin(delta_F) * R_F
    else:
        R = np.inf
        R0 = np.inf
        a = 0


    beta = np.arccos((R**2 + R0**2 - (a - wheelbase_L / 2)**2) / (2 * R * R0))

    # handle edge cases
    if np.isnan(beta):
        beta = 0

    # format output
    out = [R_F, R_R, R_FL, R_FR, R_RL, R_RR, delta_F, delta_R, R, R0, beta]
    
    return out


def inverseAckermannVelocities(radii, velocities, R0, beta, wheelRadius_r):
    """
    Description:
        Calculates angular velocities of both wheels of pseudobicycle
    Parameters:
        radii: seven element vector containing relevant radii in this order: R_F, R_R, R_FL, R_FR, R_RL, R_RR
        velocities: two element vector containing relevant angular velocities on this order: omega_FL, omega_RL
        R0: rotation radius of the center of the vehicle
        beta: angle between tangent velocity and velocity of center of the vehicle
        wheelRadius_r: front wheel radius
    Returns:
        vector consisting of angular velocities of both wheels of pseudobicycle, and forward velocity of the vehicle in this order: omega_F, omega_R, v_x
    """

    # use meaningful names
    R_F = radii[0]
    R_R = radii[1]
    R_FL = radii[2]
    R_FR = radii[3]
    R_RL = radii[4]
    R_RR = radii[5]
    omega_FL = velocities[0]
    omega_FR = velocities[1]
    omega_RL = velocities[2]
    omega_RR = velocities[3]

    # if radius is Inf, velocities are the same
    if R_F == np.inf:
        omega_F = (omega_FR + omega_FL)/2  # use avg
    else:
        omega_F = ((omega_FR * R_F / R_FR)+(omega_FL * R_F / R_FL))/2  # avg

    # if radius is Inf, velocities are the same
    if R_R == np.inf:
        omega_R = (omega_RR + omega_RL)/2  # use avg
    else:
        omega_R = ((omega_RR * R_R / R_RR)+(omega_RL * R_R / R_RL))/2  # use avg

    # calculate velocity of the vehicle
    if R0 == np.inf:  # handle edge cases
        v = (omega_FR + omega_FL + omega_RR + omega_RL)/4 * wheelRadius_r   # use avg
    else:
        Omega = (omega_FR/R_FR + omega_FL/R_FL + omega_RR/R_RR + omega_RL/R_RL)/4 * wheelRadius_r # use avg
        
        v = Omega * R0
    
    v_x = np.cos(beta) * v

    if R_F == np.inf or R_R == np.inf:
        omega_F_teor = omega_R
    else:
        omega_F_teor = omega_R*(R_F/R_R)
        
    omega_F = (omega_F + omega_F_teor)/2
    # format output
    out = [omega_F, omega_R, v_x]
    
    return out


def inverseAckermann(in_, wheeltrack_w, wheelbase_L, wheelRadius_r):
    """
    Parameters:
        in_: 12 element vector containing data reported back from the vehicle in this order: omega_FL, omega_FR, omega_RL, omega_RR, omega_FL_req, omega_FR_req, omega_RL_req, omega_RR_req, delta_FL, delta_FR, delta_RL, delta_RR
        wheeltrack_w: the vehicle's wheeltrack distance
        wheelbase_L: wheelbase of the vehicle
        wheelRadius_r: front wheel radius
    Returns:
        vector of calculated values in this order: delta_F, delta_R, omega_F, omega_F_req, R, v_x, v_x_req
    """

    # calculate angles and radii
    angles = inverseAckermannAngles(np.deg2rad(in_[8:12]), wheeltrack_w, wheelbase_L)

    # calculate measured velocities of pseudobicycle
    measuredVelocities = inverseAckermannVelocities(angles[0:6], in_[0:4], angles[9], angles[10], wheelRadius_r)

    # calculate requested velocities of pseudobicycle
    requestedVelocities = inverseAckermannVelocities(angles[0:6], in_[4:8], angles[9], angles[10], wheelRadius_r)

    # format output
    out = [angles[6], angles[7], measuredVelocities[0], requestedVelocities[0], angles[8], measuredVelocities[2], requestedVelocities[2]]
    
    return out


def car4KinematicModel(r, L, x, u):
    """
    Description:
        Function representing state space model of experimental vehicle Car4
    Parameters:
        r: front wheel radius in meters
        L: wheelbase of the vehicle in meters
        x: column vector of previous state
        u: column vector of current control
    Returns:
        x_: column vector of current derivatives of state
    """

    # when reversing, invert the wheel heading angles
    if u[2] < 0:
        u[0] = -u[0]
        u[1] = -u[1]

    x_ = np.zeros(3)

    # calculate
    x_[0] = -u[2] * r * (np.sin(x[2]) * np.cos(u[1]) * np.sin(u[0]) + np.sin(x[2]) * np.sin(u[1]) * np.cos(u[0]) - 2 * np.cos(x[2]) * np.cos(u[1]) * np.cos(u[0])) / (2 * np.cos(u[1]))
    x_[1] = u[2] * r * (2 * np.sin(x[2]) * np.cos(u[1]) * np.cos(u[0]) + np.cos(x[2]) * np.cos(u[1]) * np.sin(u[0]) - np.cos(x[2]) * np.sin(u[1]) * np.cos(u[0])) / (2 * np.cos(u[1]))
    x_[2] = u[2] * r * (np.cos(u[1]) * np.sin(u[0]) - np.sin(u[1]) * np.cos(u[0])) / (L * np.cos(u[1]))

    return x_


# data ... 15 element row vector containing reported data from the MCU in this order: req_dir, req_speed, encoder_error, omega_FL, omega_FR, omega_RL, omega_RR, omega_FL_req, omega_FR_req, omega_RL_req, omega_RR_req, delta_FL, delta_FR, delta_RL, delta_RR

class odometry:

    def __init__(self):
        self.car4_state = np.array([0, 0, 0])
        self.wheelRadius_r = 0.073
        self.wheelbase_L = 0.5
        self.wheeltrack_w = 0.4
        self.number_subscriber = rospy.Subscriber('car4_wheels_data', numpy_msg(Float32MultiArray), self.callback)
        self.position_publisher = rospy.Publisher('odometry_data', CarState, queue_size=10)
        self.last_time = 0
        self.position_msg = CarState()
        self.tf_broadcaster = tf.TransformBroadcaster()


    def callback(self, msg):
        # used to be range(4,16), but the output of the car changed

        clipped_data = np.clip(msg.data[3:15], -50, 50) #needed because of occasional mistake in data

        pseudo_bycicle_state = inverseAckermann(clipped_data, self.wheeltrack_w, self.wheelbase_L, self.wheelRadius_r)
        speed = car4KinematicModel(self.wheelRadius_r, self.wheelbase_L, self.car4_state, pseudo_bycicle_state)

        time = msg.data[0]
        delta_t = time - self.last_time
        self.last_time = time

        self.car4_state = self.car4_state + delta_t * speed
        
        self.position_msg.x = self.car4_state[0]
        self.position_msg.y = self.car4_state[1]
        self.position_msg.angle = self.car4_state[2]
        self.position_publisher.publish(self.position_msg)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.position_msg.angle)
        self.tf_broadcaster.sendTransform(
            (self.position_msg.x, self.position_msg.y, 0),  # Translation
            quaternion,
            rospy.Time.now(),
            "base_link",  # Child frame
            "odom"  # Parent frame
        )


if __name__ == '__main__':
    rospy.init_node('car4_odometry_calculator')
    odometry()
    rospy.spin()
