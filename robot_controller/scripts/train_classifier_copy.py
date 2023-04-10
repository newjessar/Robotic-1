import tensorflow as tf 
from tensorflow.keras import layers, models
import numpy as np 
import cv2
import os
import matplotlib.pyplot as plt

# Set the number of outputs/classes to train with
NR_OF_CLASSES = 6

# Set the input size of the image
IMAGE_SIZE = 28

# You can change this to stop earlier or train for longer
NR_OF_EPOCHS = 15 

# You can change the batch size of training
BATCH_SIZE = 20

## filters paprameters
n_filter = 8
w_filter = 3
h_filter = 3

train_x_data = np.load(os.environ["HOME"] + "/data/train_classifier.npy").astype(np.float32) / 255
train_y_data = np.load(os.environ["HOME"] + "/data/label_classifier.npy").astype(np.int32)

# Create a model
model = models.Sequential()

# Add CNN layer(s) to the model
model.add(layers.Conv2D(n_filter, (w_filter, h_filter), activation="relu", input_shape=(IMAGE_SIZE, IMAGE_SIZE, 3)))
# Possibly add dropout, and/or max pooling
model.add(layers.MaxPool2D(pool_size=(2,2)))

# Add a flatten layer
model.add(layers.Flatten())
# Add Dense layer(s) to the model
model.add(layers.Dense(64, activation="relu"))
# The final layer needs to be a softmax
model.add(layers.Dense(NR_OF_CLASSES, activation="softmax"))

model.compile(optimizer="adam", 
        loss="categorical_crossentropy", 
        metrics=["accuracy"])

# Train the model and store the history
history = model.fit(train_x_data, train_y_data, epochs=NR_OF_EPOCHS, batch_size = BATCH_SIZE, shuffle=True, verbose = 1)

# Plot the training accuracy over epochs
plt.plot(history.history['accuracy'])
plt.title('Model Accuracy')
plt.ylabel('Accuracy')
plt.xlabel('Epoch')
plt.legend(['Train'], loc='upper left')
plt.show()

# Save the model
save_path = os.path.join(os.environ["HOME"], "network_model")

if not os.path.exists(save_path): os.mkdir(save_path)

# This will save, and overwrite, the network model
model.save(os.path.join(save_path, "model_classifier2.h5"))
model = None
print("done")
