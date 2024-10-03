import numpy as np
import matplotlib.pyplot as plt
import random
import copy
import os


class Particle:
    def __init__(self, angle, position, choose_randomly=False):
        if choose_randomly:
            self.angle = random.uniform(*angle)
            self.position = random.choice(position)
        else:
            self.angle = angle
            self.position = position
        self.weight = 0
        
    def update_scan_values_not_optimal(self, map):
        self.angles, self.distances = generate_scan_not_optimal(self.position, self.angle, map)
        self.X = np.sin(self.angles) * self.distances
        self.Y = np.cos(self.angles) * self.distances

    def update_scan_values(self, map):
        self.angles, self.distances = generate_scan(self.position, self.angle, map)
        self.X = np.sin(self.angles) * self.distances
        self.Y = np.cos(self.angles) * self.distances

    def move(self,dx, dy, da):
        c = np.cos(self.angle)
        s = np.sin(self.angle)
        A = np.array([[c, -s],[s, c]])
        B = np.array([dx,dy])
        self.position += np.dot(A, B)
        self.angle += da

    def compare_pseudo_euclidian(self, X_scan, Y_scan):#jde to
        self.weight = 0

        X = np.array(self.X)
        Y = np.array(self.Y)
        X_scan = np.array(X_scan)
        Y_scan = np.array(Y_scan)

        differences = np.sqrt((X[:, np.newaxis] - X_scan)**2 + (Y[:, np.newaxis] - Y_scan)**2)
        distances = np.min(differences, axis=1)
        self.avg_distance = np.sum(distances)/len(distances)
        self.weight = 100 / (1 + (np.sum(distances)/(len(distances)))**2)

    def compare_distance_diff(self, distances):#lepsi nez nic
        self.weight = 1 / (1 + sum(np.sqrt((self.distances - distances)**2)))

    def kernell_multiplication(self, X_scan, Y_scan):
        self.weight = 0

    def correlation_XY(self, X_scan, Y_scan):#nic moc
        A = np.vstack((X_scan, Y_scan))
        B = np.vstack((self.X, self.Y))
        self.weight = np.corrcoef(A.flatten(), B.flatten())[0, 1]

    def correlation_dist(self, distances):#nic moc
        self.weight = np.corrcoef(self.distances, distances)[0, 1]


class Population:
    def __init__(self, pop_size):
        self.particles = [Particle(angle_range, ok_position, choose_randomly=True) for _ in range(pop_size)]

    
    def resample(self):
        elite_size = 50
        children_size = 50
        self.particles = sorted(self.particles, key=lambda particle: particle.weight, reverse=True)
        elite_particles = self.particles[0:elite_size]
        collective_weight = sum(particle.weight for particle in self.particles)
        inv_collective_weight = 1/collective_weight
        probabilities = [particle.weight*inv_collective_weight for particle in self.particles]
        cum_probabilities = np.cumsum(probabilities)
        new_particles = []
        interval_distance = 1/children_size
        start_point = random.uniform(0, interval_distance)
        current_index = 0

        for i in range(children_size):
            pointer = start_point + interval_distance * i
            while pointer > cum_probabilities[current_index]:
                current_index += 1
            new_particles.append(Particle(self.particles[current_index].angle + float(np.random.normal(0, 0.5, 1)), self.particles[current_index].position + np.random.normal(0, 2.5, size=2)))

        resampled_particles = elite_particles + new_particles
        self.particles = resampled_particles
    
    def update_scan_values(self, map):
        for particle in self.particles:
            particle.update_scan_values(map)

    def calculate_weights(self, X_scan, Y_scan):
        for particle in self.particles:
            particle.compare_pseudo_euclidian(X_scan, Y_scan)

    def move(self,dx, dy, da):
        for particle in self.particles:
            particle.move(dx, dy, da)
    

def move(pos, a, dx, dy, da):
        c = np.cos(a)
        s = np.sin(a)
        A = np.array([[c, -s],[s, c]])
        B = np.array([dx,dy])
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
        points.append([x_to_write,y_to_write])
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
        if map[y_to_write,x_to_write] == 1:
            distance = np.linalg.norm([start[0]-x_to_write, start[1]-y_to_write])
            break
        #x = x + x_step
        if D > 0:
            y = y + y_step
            D = D - 2 * abs_dx
        D = D + 2*abs_dy

    return distance

def generate_scan(pos, angle, map):
    angles = np.linspace(-(LIDAR_ANGLE/2), (LIDAR_ANGLE/2), LIDAR_NUMBER_OF_POINTS)
    distances = np.zeros(LIDAR_NUMBER_OF_POINTS)
    max_range = 400
    for i,a in enumerate(angles):
        # Calculate the direction vector based on the lidar angle
        direction = np.array([np.sin(a + angle), np.cos(a + angle)])

        end_pos = pos + max_range * direction
        max_pos = 599
        
        if end_pos[0]>max_pos or end_pos[1]>max_pos:
            range = min([np.abs((max_pos - pos[0])/direction[0]), np.abs((max_pos - pos[1])/direction[1])])
            end_pos = pos + range * direction

        end_pos = np.ceil(end_pos)
        pos = np.ceil(pos)
        pos = [int(pos[0]), int(pos[1])]
        end_pos = [int(end_pos[0]), int(end_pos[1])]

        distances[i] = bresenham_raytrace(pos, end_pos, map)

    return angles, distances
    

def generate_scan_not_optimal(pos, angle, map):
    angles = np.linspace(-np.deg2rad(LIDAR_ANGLE/2), np.deg2rad(LIDAR_ANGLE/2), LIDAR_NUMBER_OF_POINTS)

    directions = np.array([np.sin(angles + np.deg2rad(angle)), np.cos(angles + np.deg2rad(angle))])

    distances = np.zeros(LIDAR_NUMBER_OF_POINTS)
    for i in range(LIDAR_NUMBER_OF_POINTS):
        distances[i] = bresenham_raytrace(pos, directions[:, i], map)

    return angles, distances

# main

pos = [350, 300]
angle = 0
LIDAR_ANGLE = 240
LIDAR_NUMBER_OF_POINTS = 50

script_dir = os.path.dirname(os.path.realpath(__file__))
file_path = os.path.join(script_dir, '../array_data.npy')
map = np.load(file_path)

angle_range = (-np.pi, np.pi)
y, x = np.where(map == 0)
ok_position = list(zip(x,y))


angles, distances = generate_scan(pos, angle, map)
X = np.sin(angles) * distances
Y = np.cos(angles) * distances

init_pop_size = 500
pop_size = 100

population = Population(init_pop_size)

population.update_scan_values(map)
population.calculate_weights(X,Y)

dx = 0
dy = 5
da = 0.1  #kladne - doleva

pos_est = []
pos_real = []

for _ in range(0,50):
    particle_with_highest_weight = max(population.particles, key=lambda particle: particle.weight)

    pos_real.append(pos.copy())
    pos_est.append(copy.deepcopy(particle_with_highest_weight.position))

    population.resample()
    population.move(dx,dy,da)
    pos, angle = move(pos, angle, dx, dy, da)

    angles, distances = generate_scan(pos, angle, map)
    X = np.sin(angles) * distances
    Y = np.cos(angles) * distances
    population.update_scan_values(map)
    population.calculate_weights(X,Y)



particle_with_highest_weight = max(population.particles, key=lambda particle: particle.weight)
print("Particle with highest weight:")
print("Angle:", particle_with_highest_weight.angle)
print("Position:", particle_with_highest_weight.position)
print("Weight:", particle_with_highest_weight.weight)
print("")

angles = [particle.angle for particle in population.particles]
x_vis = [particle.position[0] for particle in population.particles]
y_vis = [particle.position[1] for particle in population.particles]
weights = [particle.weight for particle in population.particles]

# Plotting
# plt.figure(figsize=(8, 6))
# plt.scatter(x_vis, y_vis, c=weights, cmap='jet', alpha=0.7)
# plt.colorbar(label='Weight')
# plt.xlabel('Position')
# plt.ylabel('Angle')
# plt.title('Particle Visualization')
pos_est = np.array(pos_est)
pos_real = np.array(pos_real)
plt.scatter(pos_est[:,0],pos_est[:,1])
plt.scatter(pos_real[:,0],pos_real[:,1])
plt.imshow(map)
plt.show()

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