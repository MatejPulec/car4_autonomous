import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

# Load the image and convert it to a numpy array
with Image.open("car4_autonomous/utilities/map_mashup.pgm") as img:
    map = np.array(img)

# Define the threshold value
threshold = 50

# Set all pixels below the threshold to 0
map[map < threshold] = 0

# Save the updated map as an image
updated_img = Image.fromarray(map)
updated_img.save("car4_autonomous/utilities/map_mashup_with_threshold.pgm")

# Display the updated map
plt.imshow(updated_img, cmap='gray')
plt.show()
