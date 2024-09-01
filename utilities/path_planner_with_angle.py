import numpy as np
import matplotlib.pyplot as plt
import random
import copy

# Parameters
map_size = (600, 600)
start = [50, 50]
goal = [550, 550]
max_iterations = 5000
step_size = 10
goal_radius = 10
search_radius = 20

# Load the map
map_data = np.load('array_data.npy')

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0
        self.direction = 0
        self.angle = 0

def distance(node1, node2):
    return np.linalg.norm([node1.x - node2.x, node1.y - node2.y])

def get_random_node():
    return Node(random.randint(0, map_size[0]-1), random.randint(0, map_size[1]-1))

def is_in_collision(node):
    if map_data[node.x, node.y] == 1:
        return True
    return False

def get_nearest_node(tree, random_node):
    return min(tree, key=lambda node: distance(node, random_node))

def steer(from_node, to_node, extend_length=step_size):
    dist = distance(from_node, to_node)
    theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = from_node.x + int(extend_length * np.cos(theta))
    new_y = from_node.y + int(extend_length * np.sin(theta))
    new_node = Node(new_x, new_y)
    new_node.parent = from_node
    return new_node

def is_collision_free(node1, node2):
    # Bresenham's line algorithm or similar to check line between two nodes
    x1, y1 = node1.x, node1.y
    x2, y2 = node2.x, node2.y
    points = bresenham(x1, y1, x2, y2)
    for x, y in points:
        if map_data[x, y] == 1:
            return False
    return True

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

def get_nearby_nodes(tree, new_node):
    return [node for node in tree if distance(node, new_node) <= search_radius]

def cost_calculation(node, new_node):
    new_node_angle = np.arctan2(abs(new_node.y-node.y), abs(new_node.x-node.x))
    angle_diff = abs(node.angle -new_node_angle)
    dist = distance(new_node, node)
    curvature = 100 * np.sin(angle_diff) / dist
    # print (dist, curvature)
    return dist + curvature


def choose_parent(tree, nearby_nodes, new_node):
    if not nearby_nodes:
        return new_node.parent
    min_cost = float('inf')
    best_node = None
    for node in nearby_nodes:
        if is_collision_free(node, new_node):
            cost = node.cost + cost_calculation(node, new_node)
            if cost < min_cost:
                min_cost = cost
                best_node = node
    new_node.cost = min_cost
    return best_node

def rewire(tree, nearby_nodes, new_node):
    for node in nearby_nodes:
        if is_collision_free(new_node, node) and new_node.cost + cost_calculation(new_node, node) < node.cost:
            node.parent = new_node
            node.cost = new_node.cost + cost_calculation(new_node, node)

def get_path(last_node):
    path = []
    node = last_node
    while node is not None:
        path.append([node.x, node.y])
        node = node.parent
    return path[::-1]

def rrt_star():
    tree = [Node(start[0], start[1])]
    for _ in range(max_iterations):
        random_node = get_random_node()
        nearest_node = get_nearest_node(tree, random_node)
        new_node = steer(nearest_node, random_node)
        if is_in_collision(new_node) or not is_collision_free(nearest_node, new_node):
            continue
        nearby_nodes = get_nearby_nodes(tree, new_node)
        new_node.parent = choose_parent(tree, nearby_nodes, new_node)
        tree.append(new_node)
        rewire(tree, nearby_nodes, new_node)
        if distance(new_node, Node(goal[0], goal[1])) <= goal_radius:
            goal_node = Node(goal[0], goal[1])
            goal_node.parent = new_node
            tree.append(goal_node)
            return get_path(goal_node)
    return None

path = rrt_star()

# Plot the results
plt.figure()
plt.imshow(map_data, cmap='gray')
if path:
    path_x, path_y = zip(*path)
    plt.plot(path_y, path_x, '-r')
    plt.scatter([start[1]], [start[0]], color='blue')  # Start point
    plt.scatter([goal[1]], [goal[0]], color='green')   # Goal point
else:
    print("Path not found")
plt.title('RRT* Path Planning')
plt.show()
