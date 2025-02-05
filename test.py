import numpy as np
np.float = float  # Temporary workaround for deprecated np.float

import matplotlib.pyplot as plt

# Define an array of angles from -pi to pi
theta = np.linspace(-np.pi, np.pi, 400)

# Define sigma (controls the spread of the Gaussian)
sigma = np.pi / 4

# Compute the Gaussian weighting function
w = np.exp(-0.5 * (theta / sigma) ** 2)**1.5

# Create a polar plot
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.plot(theta, w, label=r'$\exp\left(-\frac{1}{2}(\theta/\sigma)^2\right)$')

# Customize the plot
ax.set_title("Gaussian Weighting in Polar Coordinates", va='bottom')
ax.legend()

plt.show()
