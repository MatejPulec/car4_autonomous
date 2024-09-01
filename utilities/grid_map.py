import math
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi

def file_read(filename):

    # Load data from CSV file
    data = np.loadtxt(filename, delimiter=',', skiprows=1)

    # Extract distances and angles from data
    angles = data[:, 0]
    distances = data[:, 1]

    return angles,distances

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

def SupercoverLine(start, end):
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
            if is_steep:
                points.append([y,x])
            else:
                points.append([x,y])
            y = y + y_step
            D = D - 2 * dx
        D = D + 2*dy

    if swapped:
        points.reverse()

    points = np.array(points)
    return points

from collections import deque
def flood_fill(cpoint, pmap):
    """
    cpoint: starting point (x,y) of fill
    pmap: occupancy map generated from Bresenham ray-tracing
    """
    # Fill empty areas with queue method
    sx, sy = pmap.shape
    fringe = deque()
    fringe.appendleft(cpoint)
    while fringe:
        n = fringe.pop()
        nx, ny = n
        # West
        if nx > 0:
            if pmap[nx - 1, ny] == 0.5:
                pmap[nx - 1, ny] = 0.0
                fringe.appendleft((nx - 1, ny))
        # East
        if nx < sx - 1:
            if pmap[nx + 1, ny] == 0.5:
                pmap[nx + 1, ny] = 0.0
                fringe.appendleft((nx + 1, ny))
        # North
        if ny > 0:
            if pmap[nx, ny - 1] == 0.5:
                pmap[nx, ny - 1] = 0.0
                fringe.appendleft((nx, ny - 1))
        # South
        if ny < sy - 1:
            if pmap[nx, ny + 1] == 0.5:
                pmap[nx, ny + 1] = 0.0
                fringe.appendleft((nx, ny + 1))

ang, dist = file_read("data.csv")
nonzero_indices = dist > 20
dist = dist[nonzero_indices]
ang = ang[nonzero_indices]


X = np.sin(ang) * dist + 3000
Y = np.cos(ang) * dist + 1000

X = np.rint(X/10)
Y = np.rint(Y/10)


map1 = np.ones((600, 600)) * 0.5

for i in range(0, len(X)-1):
    line = SupercoverLine((X[i], Y[i]), (X[i+1], Y[i+1]))
    for l in line:
        map1[l[0]][l[1]] = 1

line = SupercoverLine((X[len(X)-1], Y[len(X)-1]), (X[0], Y[0]))
for l in line:
    map1[l[0]][l[1]] = 1

flood_fill((500, 300), map1)

np.save('array_data.npy', map1)
plt.imshow(map1.T, origin='lower')
plt.colorbar()
plt.xlabel('x')
plt.ylabel('y')
# plt.plot(X,Y)
plt.show()
