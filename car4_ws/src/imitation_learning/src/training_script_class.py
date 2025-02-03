import tensorflow as tf
import os
import numpy as np
import pickle
import matplotlib.pyplot as plt
from sklearn.utils import shuffle


def input_to_class(turning_stick, speed_stick, y):
    if turning_stick == 0:
        y.append(0)

    if turning_stick < 0 and speed_stick < 0:
        y.append(1)

    if turning_stick > 0 and speed_stick < 0:
        y.append(2)

    if turning_stick > 0 and speed_stick > 0:
        y.append(3)

    if turning_stick < 0 and speed_stick > 0:
        y.append(4)

    return y


# Get the current script's directory
current_directory = os.path.dirname(os.path.abspath(__file__))

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
            y = input_to_class(entry["turning_stick"], entry["speed_stick"], y)

            # Add mirrored data
            mirrored_inputs = (
                list(reversed(entry["laser_ranges"])) +  # Flip laser_ranges
                # Negate goal_angle, keep goal_distance
                [-entry["goal_angle"], entry["goal_distance"]]
            )
            X.append(mirrored_inputs)

            # Negate turning_stick, keep speed_stick
            y = input_to_class(entry["turning_stick"]
                               * -1, entry["speed_stick"], y)

# Convert to NumPy arrays
X = np.array(X)
y = np.array(y)

# X, y = shuffle(X, y, random_state=42)

# i = 0
# while 1:
#     print(y[i])
#     angles = training_data[0][0]["laser_angles"]
#     goal_distance = X[i][683]
#     goal_angle = -X[i][682]
#     ranges = X[i][0:682]
#     lidar_x = np.sin(angles)*ranges
#     lidar_y = np.cos(angles)*ranges
#     goal_x = np.sin(goal_angle) * goal_distance
#     goal_y = np.cos(goal_angle) * goal_distance
#     plt.scatter(lidar_x, lidar_y)
#     plt.scatter(goal_x, goal_y, marker="*")
#     plt.axis('equal')
#     plt.show()
#     i = i+1

# Normalize the inputs
X_min = np.concatenate((np.zeros(682), [-np.pi, 0]))
X_max = np.concatenate((np.ones(682)*4, [np.pi, 3]))
X = (X - X_min) / (X_max - X_min)  # Normalize to [0, 1]


# Define the model
model = tf.keras.Sequential([
    tf.keras.layers.InputLayer(input_shape=(len(X[0]),)),  # Input layer

    # Fully connected hidden layers
    # Example: Larger layer for complexity
    tf.keras.layers.Dense(32, activation='relu'),

    # Output layer with 5 classes
    # 5 classes, softmax for probabilities
    tf.keras.layers.Dense(5, activation='softmax')
])

# Compile the model
model.compile(
    optimizer='adam',
    loss='sparse_categorical_crossentropy',  # Correct loss for classification
    metrics=['accuracy']  # Monitor accuracy
)

# Train the model
history = model.fit(
    X, y,
    validation_split=0.1,  # 20% for validation
    epochs=400,
    batch_size=1024,
    callbacks=[tf.keras.callbacks.EarlyStopping(
        patience=150, restore_best_weights=True)],
    verbose=1
)

# Evaluate the model
loss, accuracy = model.evaluate(X, y, verbose=0)
print(f"Training Loss: {loss}, Accuracy: {accuracy}")

model.save("trained_model")

# Plot the training history
plt.figure(figsize=(12, 6))
plt.plot(history.history['loss'], label='Training Loss')
plt.plot(history.history['val_loss'], label='Validation Loss')
plt.plot(history.history['accuracy'], label='Training Accuracy')
plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
plt.xlabel('Epochs')
plt.ylabel('Metrics')
plt.title('Model Training History')
plt.legend()
plt.show()

# angles = training_data[0][0][0:681]
# ranges = training_data[0][0][682:1363]
# x = np.sin(angles)*ranges
# y = np.cos(angles)*ranges
# plt.plot(x, y)
# plt.show()
