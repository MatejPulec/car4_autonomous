#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import random
import copy
from PIL import Image
import os
import rospy
from geometry_msgs.msg import Point

# Get the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Load the map
with Image.open(os.path.join(script_dir, "../map_mashup_with_safety_bubble.pgm")) as img:
    map = np.array(img)

y, x = np.where(map >= 250)
ok_position = list(zip(x, y))

# Parameters
map_size = (map.shape[0], map.shape[1])
start = [114, 500]
goal = [3072, 300]
max_iterations = 10000
step_size = 100
final_step_size = 20
exploration_bias = 0.75
extra_search_time = 0.05
ball_radius_constant = 50000
dimension = 2

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

def distance(node1, node2):
    return np.linalg.norm([node1.x - node2.x, node1.y - node2.y])

def get_random_node():
    idx = random.randint(0, len(ok_position)-1)
    return Node(ok_position[idx][0], ok_position[idx][1])

def is_in_collision(node):
    if map[node.y, node.x] == 0:
        return True
    return False

def get_nearest_node(tree, random_node):
    return min(tree, key=lambda node: distance(node, random_node))

def get_nearest_distance(tree, random_node):
    nearest_node = min(tree, key=lambda node: distance(node, random_node))
    return distance(nearest_node, random_node)


def steer(from_node, to_node, extend_length=step_size):
    dist = distance(from_node, to_node)
    if dist <= extend_length:
        return to_node
    theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = from_node.x + int(extend_length * np.cos(theta))
    new_y = from_node.y + int(extend_length * np.sin(theta))
    new_node = Node(new_x, new_y)
    new_node.parent = from_node
    return new_node

def is_collision_free(node1, node2):
    x1, y1 = node1.x, node1.y
    x2, y2 = node2.x, node2.y
    points = bresenham(x1, y1, x2, y2)
    for x, y in points:
        if map[y, x] <= 240:  # Note: map_data[y, x] due to image coordinate system
            return False
    return True

def find_collision_node(node1, node2):
    x1, y1 = node1.x, node1.y
    x2, y2 = node2.x, node2.y
    points = bresenham(x1, y1, x2, y2)
    old_x = x1
    old_y = y1
    for x, y in points:
        if map[y, x] <= 240:  # Note: map_data[y, x] due to image coordinate system
            return Node(old_x, old_y)
        old_x = x
        old_y = y
        

def bresenham(x1, y1, x2, y2):
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy
    while True:
        points.append((x1, y1))
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
    return points

def get_nearby_nodes(tree, new_node, radius):
    return [node for node in tree if distance(node, new_node) <= radius]

def choose_parent(tree, nearby_nodes, new_node):
    if not nearby_nodes:
        return new_node.parent
    min_cost = float('inf')
    best_node = None
    for node in nearby_nodes:
        if is_collision_free(node, new_node):
            cost = node.cost + distance(node, new_node)
            if cost < min_cost:
                min_cost = cost
                best_node = node
    new_node.cost = min_cost
    return best_node

def rewire(tree, nearby_nodes, new_node):
    for node in nearby_nodes:
        if is_collision_free(new_node, node) and new_node.cost + distance(new_node, node) < node.cost:
            node.parent = new_node
            node.cost = new_node.cost + distance(new_node, node)

def get_path(last_node):
    path = []
    node = last_node
    while node is not None:
        path.append([int(node.x), int(node.y)])
        node = node.parent
    return path[::-1]

def check_the_cost(tree):
    final_node = Node(goal[0], goal[1])
    nearby_nodes = get_nearby_nodes(tree, final_node, step_size)
    if len(nearby_nodes) == 0:
        return 404
    final_node.parent = choose_parent(tree, nearby_nodes, final_node)
    return get_path(final_node)

def check_coordinates_collision(coordinates, map):
    for x, y in coordinates:
        if map[int(y), int(x)] <= 240:  # Note: map_data[y, x] due to image coordinate system
            return False
    return True

def smooth_out_path(path):
    last_point_idx = len(path) - 1
    for index, point in enumerate(path):
        if index != 0 and index != last_point_idx:
            coordinates = bresenham(path[index-1][0], path[index-1][1], path[index+1][0], path[index+1][1])

            should_update = check_coordinates_collision(coordinates, map)

            if should_update:
                point[0] = np.round((path[index-1][0] + path[index+1][0]) / 2)
                point[1] = np.round((path[index-1][1] + path[index+1][1]) / 2)
    return path

def straighten_out_path(path):
    last_point_idx = len(path) - 1
    index_start = 0
    index_end = last_point_idx
    while index_start < last_point_idx:
        coordinates = bresenham(path[index_start][0], path[index_start][1], path[index_end][0], path[index_end][1])
        if check_coordinates_collision(coordinates, map) == True:
            N = np.ceil(np.linalg.norm([path[index_start][0] - path[index_end][0], path[index_start][1] - path[index_end][1]]) / final_step_size)
            delta_0 = (path[index_end][0] - path[index_start][0]) / N
            delta_1 = (path[index_end][1] - path[index_start][1]) / N
            for n in range(int(N)):
                path[index_start + n][0] = np.round(path[index_start][0] + n * delta_0)
                path[index_start + n][1] = np.round(path[index_start][1] + n * delta_1)
            path[int(index_start+N) : int(index_end)] = []
            last_point_idx = len(path) - 1
            index_start=index_start + int(N)
            index_end = last_point_idx
            continue
        index_end = index_end-1
        if index_end == index_start:
            index_start = index_start+1
            index_end = last_point_idx
    return path

def interpolate_path(path, n):
    new_path = []
    
    for i in range(len(path) - 1):
        # Get the start and end points
        x_start, y_start = path[i]
        x_end, y_end = path[i + 1]
        
        # Generate n points, including the start and end points
        for t in np.linspace(0, 1, n):
            x_interp = x_start + t * (x_end - x_start)
            y_interp = y_start + t * (y_end - y_start)
            
            # Round the interpolated points to the nearest integer (optional, depending on application)
            new_path.append([round(x_interp), round(y_interp)])
    
    # Add the last point of the original path, since the loop only adds intermediate points
    new_path.append(path[-1])
    
    return new_path



def rrt_star(exploration_bias, extra_search_time):
    old_path = []
    tree = [Node(start[0], start[1])]
    final_node = Node(goal[0], goal[1])
    final_iter = max_iterations
    for i in range(max_iterations):
        if random.random() < exploration_bias:
            random_node = final_node
        else:
            random_node = get_random_node()
        nearest_node = get_nearest_node(tree, random_node)
        new_node = steer(nearest_node, random_node)
        if is_in_collision(new_node) or not is_collision_free(nearest_node, new_node):
            continue
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + distance(new_node, nearest_node)
        if new_node.x == final_node.x and new_node.y == final_node.y:
            exploration_bias = 0
        rewire_radius = np.minimum(step_size, (ball_radius_constant * (np.log(i)/i))**(1/dimension))
        nearby_nodes = get_nearby_nodes(tree, new_node, rewire_radius)
        new_node.parent = choose_parent(tree, nearby_nodes, new_node)
        tree.append(new_node)
        rewire(tree, nearby_nodes, new_node)
        # print(get_nearest_distance(tree, final_node))
        # print(i)
        if check_the_cost(tree) != 404 and final_iter == max_iterations:
            final_iter = np.round(i * (1+extra_search_time))

        if i >= final_iter:
            return tree, None
    return tree, None

def goal_callback(msg):
    global goal
    goal = [msg.x, msg.y]  # Update global goal coordinates
    rospy.loginfo(f"Goal coordinates updated: {goal}")
    create_trajectory()  # Call the trajectory function when a new goal is received

def create_trajectory():
    if goal is None:
        return  # Ensure goal is set before attempting to calculate the trajectory
    
    tree, path = rrt_star(exploration_bias, extra_search_time)

    final_node = Node(goal[0], goal[1])
    nearby_nodes = get_nearby_nodes(tree, final_node, step_size)
    final_node.parent = choose_parent(tree, nearby_nodes, final_node)
    if final_node.parent is not None:
        tree.append(final_node)

    path = get_path(final_node)

    path = interpolate_path(path, 5)

    for _ in range(3):
        path = smooth_out_path(path)
        path = straighten_out_path(path)
    
    # Plot the results
    path_x = [point[0] for point in path]
    path_y = [point[1] for point in path]

    plt.imshow(map)
    plt.scatter(path_x, path_y)
    plt.show()

def main():
    rospy.init_node('trajectory_planner', anonymous=True)
    rospy.Subscriber('/goal_coordinates', Point, goal_callback)  # Subscribe to the goal coordinates topic
    rospy.spin()  # Keep the node running



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass