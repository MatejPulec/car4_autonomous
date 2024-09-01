import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

def create_circular_mask(radius):
    diameter = 2 * radius + 1
    mask = np.zeros((diameter, diameter), dtype=np.uint8)
    center = radius
    for i in range(diameter):
        for j in range(diameter):
            if (i - center) ** 2 + (j - center) ** 2 <= radius ** 2:
                mask[i, j] = 1
    return mask

def apply_obstacle_radius_with_mask(map, obstacle, mask, radius):
    height, width = map.shape
    mask_diameter = mask.shape[0]
    
    for x, y in obstacle:
        x_start = max(0, x - radius)
        y_start = max(0, y - radius)
        x_end = min(width, x + radius + 1)
        y_end = min(height, y + radius + 1)
        
        for i in range(x_start, x_end):
            for j in range(y_start, y_end):
                if map[j, i] != 0:
                    mask_x = i - (x - radius)
                    mask_y = j - (y - radius)
                    if mask[mask_y, mask_x]:
                        map[j, i] = 0
    return map

# Load the image and convert it to a numpy array
with Image.open("map_lifelong_HR (copy).pgm") as img:
    map = np.array(img)

# Find the coordinates of obstacle pixels
y, x = np.where(map == 0)
obstacle = list(zip(x, y))

# Define the radius around each obstacle pixel
radius = 10

# Create the circular mask
circular_mask = create_circular_mask(radius)

# Apply the obstacle radius with the circular mask
updated_map = apply_obstacle_radius_with_mask(map, obstacle, circular_mask, radius)

# Save the updated map as an image
updated_img = Image.fromarray(updated_map)
updated_img.save("map_with_safety_bubble.pgm")

plt.imshow(updated_img, cmap='gray')
plt.show()