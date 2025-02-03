import tensorflow as tf
import os
import numpy as np
import pickle
import matplotlib.pyplot as plt
from sklearn.utils import shuffle

# Get the current script's directory
current_directory = os.path.dirname(os.path.abspath(__file__))

# List of training data file paths
# file_names = [
#     "../training_data/training_data_manual_20_01.pkl",
#     "../training_data/training_data_manual_20_01_2.pkl",
#     "../training_data/training_data_disparity_20_01.pkl",
# ]

file_names = [
    "../training_data/training_data_empty_hallway.pkl",
    "../training_data/training_data_empty_hallway_2.pkl",
    "../training_data/training_data_empty_hallway_3.pkl",
    "../training_data/training_data_sides.pkl",
    "../training_data/training_data_sides_2.pkl",
    "../training_data/training_data_sides_3.pkl",
    "../training_data/training_data_sides_4.pkl",
    "../training_data/training_data_hallway_obstacles.pkl",
    "../training_data/training_data_hallway_obstacles_2.pkl",
    "../training_data/training_data_hallway_obstacles_3.pkl",
    "../training_data/training_data_hallway_obstacles_4.pkl",
    "../training_data/training_data_hallway_obstacles_5.pkl",
]

X = []
y = []

for file_name in file_names:
    # Construct the full file path including the filename
    file_path = os.path.join(current_directory, file_name)

    # Check if the file exists before loading
    if not os.path.exists(file_path):
        raise FileNotFoundError(
            f"Training data file not found at: {file_path}")

    # Log the resolved path for debugging
    print(f"Loading training data from: {file_path}")

    # Load the training data
    training_data = np.load(file_path, allow_pickle=True)

    for batch in training_data:
        for entry in batch:

            # skip if no input
            if entry["speed_stick"] == 0:
                continue

            # Flatten and concatenate inputs
            inputs = (
                entry["laser_ranges"] +  # Flatten laser_ranges
                [entry["goal_angle"], entry["goal_distance"]]  # Goal data
            )
            X.append(inputs)

            # Outputs (targets)
            targets = [entry["speed_stick"], entry["turning_stick"]]
            y.append(targets)

            # Add mirrored data
            mirrored_inputs = (
                list(reversed(entry["laser_ranges"])) +  # Flip laser_ranges
                # Negate goal_angle, keep goal_distance
                [-entry["goal_angle"], entry["goal_distance"]]
            )
            X.append(mirrored_inputs)

            # Negate turning_stick, keep speed_stick
            mirrored_targets = [entry["speed_stick"], -entry["turning_stick"]]
            y.append(mirrored_targets)

# Convert to NumPy arrays
X = np.array(X)
y = np.array(y)

X, y = shuffle(X, y, random_state=42)

# Normalize the inputs
X_min = np.concatenate((np.zeros(682), [-np.pi, 0]))
X_max = np.concatenate((np.ones(682)*4, [np.pi, 3]))
X = (X - X_min) / (X_max - X_min)  # Normalize to [0, 1]


# Define the model
model = tf.keras.Sequential([
    tf.keras.layers.InputLayer(input_shape=(len(X[0]),)),  # Input layer

    # Fully connected hidden layers
    tf.keras.layers.Dense(4, activation='relu'),


    tf.keras.layers.Dense(2)                               # Output layer
])

# Compile the model
model.compile(optimizer='adam', loss='mse', metrics=['mae'])

# Train the model
history = model.fit(
    X, y,
    validation_split=0.2,  # 20% for validation
    epochs=400,
    batch_size=1024,
    callbacks=[tf.keras.callbacks.EarlyStopping(
        patience=200, restore_best_weights=True)],
    verbose=1
)

# Evaluate the model
loss, mae = model.evaluate(X, y, verbose=0)
print(f"Training Loss: {loss}, Mean Absolute Error: {mae}")

model.save("trained_model")

# Plot the training history
plt.figure(figsize=(12, 6))
plt.plot(history.history['loss'], label='Training Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.xlabel('Epochs')
plt.ylabel('Loss')
plt.title('Model Training History')
plt.legend()
plt.show()

# angles = training_data[0][0][0:681]
# ranges = training_data[0][0][682:1363]
# x = np.sin(angles)*ranges
# y = np.cos(angles)*ranges
# plt.plot(x, y)
# plt.show()
