from PIL import Image
import numpy as np

# Example usage
input_image = "map_mashup2.png"  # Path to the input image
output_image = "output_image.pgm"  # Path to save the modified image as .pgm

# Open the image and convert it to grayscale (if it's not already)
image = Image.open(input_image).convert("L")

# Convert image to a NumPy array
image_array = np.array(image)

# Create a new array to store the modified image
new_image_array = np.array(image)

# Override all pixels according to the threshold conditions
new_image_array[image_array < 150] = 0
new_image_array[(image_array >= 150) & (image_array <= 210)] = 205
new_image_array[image_array > 210] = 255

# Convert the array back to an image
modified_image = Image.fromarray(new_image_array)

# Save the modified image as .pgm (Portable Gray Map)
modified_image.save(output_image, format="PPM")
print(f"Image saved as {output_image}")
