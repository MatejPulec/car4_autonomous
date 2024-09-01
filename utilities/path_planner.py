import numpy as np
import matplotlib.pyplot as plt
import random
import copy
from PIL import Image

# Parameters
map_size = (600, 600)
start = [500, 200]
goal = [500, 350]
max_iterations = 5000
step_size = 10
goal_radius = 10
search_radius = 20

# Load the map
with Image.open("map_lifelong_HR (copy).pgm") as img:
    map_data = np.array(img)
max_pos = [map_data.shape[1]-1, map_data.shape[0]-1]
min_pos = [0, 0]

angle_range = (-np.pi, np.pi)
y, x = np.where(map_data == 255)
ok_position = list(zip(x, y))

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

def distance(node1, node2):
    return np.linalg.norm([node1.x - node2.x, node1.y - node2.y])

def get_random_node():
    return Node(random.randint(0, map_size[0]-1), random.randint(0, map_size[1]-1))

def is_in_collision(node):
    if map_data[node.y, node.x] == 0:
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
    x1, y1 = node1.x, node1.y
    x2, y2 = node2.x, node2.y
    points = bresenham(x1, y1, x2, y2)
    for x, y in points:
        if map_data[y, x] == 0:  # Note: map_data[y, x] due to image coordinate system
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
        path.append([node.x, node.y])
        node = node.parent
    return path[::-1]

def rrt_star():
    tree = [Node(start[0], start[1])]
    edges = []  # To store edges for plotting the whole tree
    for _ in range(max_iterations):
        random_node = get_random_node()
        nearest_node = get_nearest_node(tree, random_node)
        new_node = steer(nearest_node, random_node)
        if is_in_collision(new_node) or not is_collision_free(nearest_node, new_node):
            continue
        nearby_nodes = get_nearby_nodes(tree, new_node)
        new_node.parent = choose_parent(tree, nearby_nodes, new_node)
        tree.append(new_node)
        edges.append((new_node, nearest_node))  # Store the edge
        rewire(tree, nearby_nodes, new_node)
        if distance(new_node, Node(goal[0], goal[1])) <= goal_radius:
            goal_node = Node(goal[0], goal[1])
            goal_node.parent = new_node
            tree.append(goal_node)
            edges.append((goal_node, new_node))  # Store the edge
            return tree, edges, get_path(goal_node)
    return tree, edges, None

tree, edges, path = rrt_star()

# Plot the results
plt.figure()
plt.imshow(map_data, cmap='gray')

# Plot the whole tree
for edge in edges:
    node1, node2 = edge
    plt.plot([node1.x, node2.x], [node1.y, node2.y], '-g', alpha=0.5)

# Plot the path
if path:
    path_x, path_y = zip(*path)
    plt.plot(path_y, path_x, '-r', linewidth=2)
    plt.scatter([start[0]], [start[1]], color='blue')  # Start point
    plt.scatter([goal[0]], [goal[1]], color='green')   # Goal point
else:
    print("Path not found")

plt.title('RRT* Path Planning with Tree')
plt.show()
