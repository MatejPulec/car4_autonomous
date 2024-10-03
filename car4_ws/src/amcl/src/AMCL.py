#!/usr/bin/env python3
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from odometry.msg import CarState
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
import random
import copy
from PIL import Image
import tf
import os
import logging
import threading

# Configure the logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')



class Laser:
    def __init__(self):
        self.angles = []

    def callback(self, msg):
        self.ranges = msg.ranges
        if len(self.angles) == 0:
            self.angles = np.arange(
                msg.angle_max, msg.angle_min - msg.angle_increment, -msg.angle_increment)
            pass


class Odometry:
    def callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.angle = msg.angle


class Particle:
    def __init__(self, angle, position, choose_randomly=False):
        if choose_randomly:
            self.angle = random.uniform(*angle)
            self.position = random.choice(position)
        else:
            self.angle = angle
            self.position = position
        self.weight = 0

    def update_scan_values(self, map, resolution, LIDAR_NUMBER_OF_POINTS, LIDAR_ANGLE, no_zeros=True):
        self.angles, self.distances = generate_scan(
            self.position, self.angle, map, resolution, LIDAR_NUMBER_OF_POINTS, LIDAR_ANGLE)
        if no_zeros:
            nonzero_idx = np.nonzero(self.distances >= 0.05)
        self.X = np.sin(self.angles[nonzero_idx]) * self.distances[nonzero_idx]
        self.Y = np.cos(self.angles[nonzero_idx]) * self.distances[nonzero_idx]

    def move(self, dx, dy, da, max_pos, min_pos, resolution, proportional_error):
        dy = -dy        # different axes compensation
        c = np.cos(self.angle)
        s = np.sin(self.angle)
        A = np.array([[c, -s], [s, c]])
        B = np.array([dx*np.random.normal(1, proportional_error, 1)[0],
                     dy*np.random.normal(1, proportional_error, 1)[0]])
        self.position += np.dot(A, B) / resolution

        if self.position[0] > max_pos[0]:
            self.position[0] = max_pos[0]

        if self.position[0] < min_pos[0]:
            self.position[0] = min_pos[0]

        if self.position[1] > max_pos[1]:
            self.position[1] = max_pos[1]

        if self.position[1] < min_pos[1]:
            self.position[1] = min_pos[1]

        self.angle -= da*np.random.normal(1, proportional_error, 1)[0]

    def compare_pseudo_euclidian(self, X_scan, Y_scan):  # jde to

        X = np.array(self.X)
        Y = np.array(self.Y)

        X_scan = np.array(X_scan)
        Y_scan = np.array(Y_scan)

        if len(X) <= 5 and len(X_scan) <= 20:
            pass
        elif len(X) <= 5:
            self.weight = self.weight * 0.5
        elif len(X_scan) == 0:
            pass
        else:
            differences = np.sqrt(
                (X[:, np.newaxis] - X_scan)**2 + (Y[:, np.newaxis] - Y_scan)**2)
            distances = np.min(differences, axis=1)
            self.avg_distance = np.sum(distances)/len(distances)
            self.weight = 100 / (1 + 10*self.avg_distance)

    def compare_distance_diff(self, distances):  # lepsi nez nic
        self.weight = 1 / (1 + sum(np.sqrt((self.distances - distances)**2)))

    def kernell_multiplication(self, X_scan, Y_scan):
        self.weight = 0

    def correlation_XY(self, X_scan, Y_scan):  # nic moc

        A = np.vstack((X_scan, Y_scan))
        B = np.vstack((self.X, self.Y))
        self.weight = np.corrcoef(A.flatten(), B.flatten())[0, 1]

    def correlation_dist(self, distances):  # nic moc
        self.weight = np.corrcoef(self.distances, distances)[0, 1]


class Population:
    def __init__(self, pop_size, position, angle, choose_randomly):
        self.particles = [Particle(
            angle, position, choose_randomly=choose_randomly) for _ in range(pop_size)]

    def sort(self):
        self.particles = sorted(
            self.particles, key=lambda particle: particle.weight, reverse=True)

    def resample(self, pos_sigma, angle_sigma, elite_size, children_size):
        self.particles = sorted(
            self.particles, key=lambda particle: particle.weight, reverse=True)
        elite_particles = self.particles[0:elite_size]
        collective_weight = sum(particle.weight for particle in self.particles)
        inv_collective_weight = 1/collective_weight
        probabilities = [particle.weight *
                         inv_collective_weight for particle in self.particles]
        cum_probabilities = np.cumsum(probabilities)
        new_particles = []
        # interval_distance = 1/children_size
        interval_distance = 0
        start_point = random.uniform(0, interval_distance)
        current_index = 0

        for i in range(children_size):
            pointer = start_point + interval_distance * i
            while pointer > cum_probabilities[current_index]:
                current_index += 1
            new_particles.append(Particle(self.particles[current_index].angle + float(np.random.normal(
                0, angle_sigma, 1)), self.particles[current_index].position + np.random.normal(0, pos_sigma, size=2)))

        resampled_particles = elite_particles + new_particles
        self.particles = resampled_particles

    def update_scan_values(self, map, resolution, LIDAR_NUMBER_OF_POINTS, LIDAR_ANGLE, no_zeros):
        for particle in self.particles:
            particle.update_scan_values(
                map, resolution, LIDAR_NUMBER_OF_POINTS, LIDAR_ANGLE, no_zeros)

    def calculate_weights(self, X_scan, Y_scan):
        for particle in self.particles:
            particle.compare_pseudo_euclidian(X_scan, Y_scan)

    def move(self, dx, dy, da, max_pos, min_pos, resolution, proportional_error):
        for particle in self.particles:
            particle.move(dx, dy, da, max_pos, min_pos,
                          resolution, proportional_error)


def move(pos, a, dx, dy, da):
    c = np.cos(a)
    s = np.sin(a)
    A = np.array([[c, -s], [s, c]])
    B = np.array([dx, dy])
    pos += np.dot(A, B)
    a += da

    return pos, a


def bresenham(start, end):
    x1, y1 = start
    x2, y2 = end
    x1, y1, x2, y2 = np.ceil(x1), np.ceil(y1), np.ceil(x2), np.ceil(y2)
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = abs(x2 - x1)  # recalculate differentials
    dy = abs(y2 - y1)  # recalculate differentials

    if y2 > y1:
        y_step = 1
    else:
        y_step = -1

    points = []
    x = x1
    y = y1
    D = 2 * dy - dx
    for x in range(x1, x2 + 1):
        x_to_write = x
        y_to_write = y
        if is_steep:
            x_to_write, y_to_write = y_to_write, x_to_write
        points.append([x_to_write, y_to_write])
        x = x + 1
        if D > 0:
            y = y + y_step
            D = D - 2 * dx
        D = D + 2*dy

    if swapped:
        points.reverse()

    points = np.array(points)
    return points


def bresenham_raytrace(start, end, map):
    distance = 0
    x1, y1 = start
    x2, y2 = end
    is_steep = abs(y2 - y1) > abs(x2 - x1)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # swap start and end points if necessary and store swap state

    if x2 > x1:
        x_step = 1
    else:
        x_step = -1
    if y2 > y1:
        y_step = 1
    else:
        y_step = -1

    abs_dx = np.abs(x2 - x1)
    abs_dy = np.abs(y2 - y1)

    x = x1
    y = y1
    D = 2 * abs_dy - abs_dx
    for x in range(x1, x2 + x_step, x_step):
        x_to_write = x
        y_to_write = y
        if is_steep:
            x_to_write, y_to_write = y_to_write, x_to_write
        if map[y_to_write, x_to_write] == 0:
            distance = np.linalg.norm(
                [start[0]-x_to_write, start[1]-y_to_write])
            break
        # x = x + x_step
        if D > 0:
            y = y + y_step
            D = D - 2 * abs_dx
        D = D + 2*abs_dy

    return distance


def generate_scan(pos, angle, map, resolution, LIDAR_NUMBER_OF_POINTS, LIDAR_ANGLE):
    angles = np.linspace(-(LIDAR_ANGLE/2), (LIDAR_ANGLE/2),
                         LIDAR_NUMBER_OF_POINTS)
    angles = np.deg2rad(angles)
    distances = np.zeros(LIDAR_NUMBER_OF_POINTS)
    max_range = 4 / resolution
    compensation_angle = 0
    for i, a in enumerate(angles):
        # Calculate the direction vector based on the lidar angle
        direction = np.array(
            [np.cos(a + angle + compensation_angle), np.sin(a + angle + compensation_angle)])

        end_pos = pos + max_range * direction
        max_pos = [map.shape[1]-1, map.shape[0]-1]
        min_pos = [0, 0]

        range = []

        if end_pos[0] > max_pos[0]:
            range.append(np.abs((max_pos[0] - pos[0])/direction[0]))

        if end_pos[0] < min_pos[0]:
            range.append(np.abs((min_pos[0] - pos[0])/direction[0]))

        if end_pos[1] > max_pos[1]:
            range.append(np.abs((max_pos[1] - pos[1])/direction[1]))

        if end_pos[1] < min_pos[1]:
            range.append(np.abs((min_pos[1] - pos[1])/direction[1]))

        if len(range) != 0:
            end_pos = pos + min(range) * direction

        end_pos = np.ceil(end_pos)
        pos = np.ceil(pos)
        pos = [int(pos[0]), int(pos[1])]
        end_pos = [int(end_pos[0]), int(end_pos[1])]

        distances[i] = resolution * bresenham_raytrace(pos, end_pos, map)
    return angles, distances


# Global variables for the latest particle position and angle
latest_position = None
latest_angle = None

def send_tf_transformation(tf_broadcaster, resolution):
    global latest_position, latest_angle
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if latest_position is not None and latest_angle is not None:
            quaternion = tf.transformations.quaternion_from_euler(0, 0, latest_angle * -1)
            tf_broadcaster.sendTransform(
                # Translation
                (latest_position[0] * resolution,
                 latest_position[1] * resolution * -1, 0),#different axes
                quaternion,
                rospy.Time.now(),
                "AMCL",  # Child frame
                "map"  # Parent frame
            )
        rate.sleep()

def main():
    # SETUP
    LIDAR_ANGLE = 240
    LIDAR_NUMBER_OF_POINTS = 70  # 682
    init_pop_size = 500
    pop_size = 100
    no_zeros = True  # delete zeros in scan
    resolution = 0.025  # resolution of map
    proportional_error = 0.05
    pos_sigma = 5
    angle_sigma = 0.25
    elite_size = 5
    children_size = 15

    script_dir = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(script_dir, '../map_mashup.pgm')

    global latest_position, latest_angle

    # Load map
    with Image.open(file_path) as img:
        map = np.array(img)
    max_pos = [map.shape[1]-1, map.shape[0]-1]
    min_pos = [0, 0]

    angle_range = (-np.pi, np.pi)
    y, x = np.where(map == 255)
    ok_position = list(zip(x, y))

    # Transmit, recieve init
    tf_broadcaster = tf.TransformBroadcaster()

    position_publisher = rospy.Publisher(
        'AMCL_position', PoseStamped, queue_size=10)

    # Population init
    position = [114, 500]
    angle = 0
    population = Population(1, angle, position, choose_randomly=False)
    population.particles[0].position = [114, 500]
    population.particles[0].angle = 0

    # Odometry init
    odometry_msg = rospy.wait_for_message('odometry_data', CarState)
    car4_odom_state = np.array(
        [odometry_msg.x, odometry_msg.y, odometry_msg.angle])

    laser_angles = []
    pos_est = []
    pos_real = []

    # # Create a 4x4 subplot grid
    # fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(4, 4))

    # Create a TF broadcaster
    tf_broadcaster = tf.TransformBroadcaster()

    # Start the transformation sending thread
    tf_thread = threading.Thread(target=send_tf_transformation, args=(tf_broadcaster, resolution))
    tf_thread.start()

    a = 400
    while a:
        a -= 1
        old_car4_odom_state = car4_odom_state

        odometry_msg = rospy.wait_for_message('odometry_data', CarState)
        car4_odom_state = np.array(
            [odometry_msg.x, odometry_msg.y, odometry_msg.angle])
        car4_odom_dx = car4_odom_state[0]-old_car4_odom_state[0]
        car4_odom_dy = car4_odom_state[1]-old_car4_odom_state[1]
        car4_odom_da = car4_odom_state[2]-old_car4_odom_state[2]

        c = np.cos(old_car4_odom_state[2])
        s = np.sin(old_car4_odom_state[2])
        dx_local = car4_odom_dx * c + car4_odom_dy * s
        dy_local = car4_odom_dx * -s + car4_odom_dy * c

        laser_msg = rospy.wait_for_message('/scan', LaserScan)
        laser_ranges = laser_msg.ranges
        if len(laser_angles) == 0:
            laser_angles = np.arange(laser_msg.angle_max, laser_msg.angle_min -
                                     laser_msg.angle_increment, -laser_msg.angle_increment)

        laser_ranges = np.array(laser_ranges)
        
        jumps = 0
        for i in range(len(laser_ranges)-1):
            if abs(laser_ranges[i]-laser_ranges[i+1])>0.5:
                jumps += 1

        if no_zeros:
            nonzero_idx = np.nonzero(laser_ranges >= 0.05)
        X = np.sin(laser_angles[nonzero_idx]) * laser_ranges[nonzero_idx]
        Y = np.cos(laser_angles[nonzero_idx]) * laser_ranges[nonzero_idx]


        population.move(dx_local, dy_local, car4_odom_da,
                        max_pos, min_pos, resolution, proportional_error)
        population.update_scan_values(
            map, resolution, LIDAR_NUMBER_OF_POINTS, LIDAR_ANGLE, no_zeros)
        population.calculate_weights(X, Y)  # todo use position error
        population.sort()

        # plt.scatter(X,Y)
        # plt.scatter(population.particles[0].X, population.particles[0].Y)
        # plt.show()


        # # Example data
        # particles_to_plot = min(4, len(population.particles))
        # X_particle = [particle.X for particle in population.particles[:particles_to_plot]]
        # Y_particle = [particle.Y for particle in population.particles[:particles_to_plot]]
        # Weight_particle = [particle.weight for particle in population.particles[:particles_to_plot]]

        # # Iterate over the first particles_to_plot particles and plot them in the subplots
        # for i, ax in enumerate(axes.flat):
        #     if i < particles_to_plot:
        #         ax.clear()
        #         ax.scatter(X, Y)
        #         ax.scatter(X_particle[i], Y_particle[i])
        #         ax.set_title(f'Particle {i + 1}\nWeight: {Weight_particle[i]:.4f}\nJumps: {jumps}')
        #         ax.axis('equal')

        # # Adjust layout to prevent clipping of titles
        # plt.tight_layout()

        # # Show the plot
        # plt.pause(0.1)

        
        # x_est = [sublist[0] for sublist in pos_est]
        # y_est = [sublist[1] for sublist in pos_est]
        # plt.clf()
        # plt.imshow(map)
        # plt.plot(x_est, y_est)
        # plt.pause(0.001)

        particle_with_highest_weight = max(
            population.particles, key=lambda particle: particle.weight)

        pos_real.append(car4_odom_state.copy())
        pos_est.append(copy.deepcopy(particle_with_highest_weight.position))

        if jumps > 4:
            population.resample(pos_sigma, angle_sigma, elite_size, children_size)  #todo calculate children size on the fly, maybe add random particles?

        latest_position = copy.deepcopy(particle_with_highest_weight.position)
        latest_angle = copy.deepcopy(particle_with_highest_weight.angle)

        # quaternion = tf.transformations.quaternion_from_euler(
        #     0, 0, particle_with_highest_weight.angle)
        # tf_broadcaster.sendTransform(
        #     # Translation
        #     (particle_with_highest_weight.position[0]*resolution,
        #      particle_with_highest_weight.position[1]*resolution, 0),
        #     quaternion,
        #     rospy.Time.now(),
        #     "AMCL",  # Child frame
        #     "map"  # Parent frame
        # )

        # angles = [particle.angle for particle in population.particles]
        # x_vis = [particle.position[0] for particle in population.particles]
        # y_vis = [particle.position[1] for particle in population.particles]
        # weights = [particle.weight for particle in population.particles]

        # Plotting
        # plt.figure(figsize=(20, 10))
        # plt.scatter(x_vis[0: min(len(x_vis), 11)], y_vis[0: min(len(x_vis), 11)], c=weights[0: min(len(x_vis), 11)], cmap='jet', alpha=0.7)
        # plt.colorbar(label='Weight')
        # plt.xlabel('Position')
        # plt.ylabel('Angle')
        # plt.title('Particle Visualization')
        # plt.imshow(map)

        # plt.show()

        # particles[0].position = [300, 300]
        # particles[0].angle = 45
        # particles[0].update_scan_values(map)
        # print(particles[0].position)

        # plt.plot(particles[0].X, particles[0].Y)
        # plt.show()

        # # # plt.plot(X,Y)
        # # # plt.axis('equal')
        # # # plt.imshow(map.T, origin='lower')
        # # # plt.colorbar()
        # # # plt.show()

    # plt.imshow(map)
    # x_est = [sublist[0] for sublist in pos_est]
    # y_est = [sublist[1] for sublist in pos_est]
    # plt.plot(x_est, y_est)
    plt.show()
    tf_thread.join()


if __name__ == '__main__':
    rospy.init_node('AMCL')
    main()
    rospy.spin()
