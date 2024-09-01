import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

# Load the image and convert it to a numpy array
with Image.open("map_with_safety_bubble.pgm") as img:
    map = np.array(img)

plt.imshow(map, cmap='gray')
plt.show()