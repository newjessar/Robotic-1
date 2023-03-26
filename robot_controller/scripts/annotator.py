import cv2
import os
import numpy as np 
import sys

# Global variables
mouse_x = 0
mouse_y = 0

# Set the number of classes 
NR_OF_CLASSES = 6

# Size of the input images of our network
BOUNDING_BOX_IMAGE_SIZE = 28

data_x = [] # List containing the images
data_y = [] # List containing the labels

# Get the mouse position when hovering over the image
def on_mouse(event, x, y, frame, param):
    global mouse_x, mouse_y
    mouse_x = x
    mouse_y = y

# Need to give the name of dataset to load for annotating
if len(sys.argv) != 2:
    print("Give name of dataset without .npy")
    exit()

# Data should be stored in /home/username/data/
path = os.path.join(os.environ["HOME"], "data")
dataset = sys.argv[1] 
data = np.load(os.path.join(path, "{0}.npy".format(dataset)), allow_pickle=True)

# If data was saved previously, load it so we can append to it
if os.path.exists(os.path.join(path, "train_classifier.npy") and os.path.join(path, "label_classifier.npy")):
    data_x = list(np.load(os.path.join(path, "train_classifier.npy")))
    data_y = list(np.load(os.path.join(path, "label_classifier.npy")))
    print("Loaded data containing {0} items".format(len(data_x)))

cv2.namedWindow("bounding boxes")
cv2.setMouseCallback("bounding boxes", on_mouse)

# For each image in the dataset
bounding_boxes = data
image_list = []
columns = int(np.sqrt(len(bounding_boxes)))
columns = 5
rows = 5
count = 0 


print ("Nr of bounding boxes: ", len(bounding_boxes))
while count < len(bounding_boxes):
    image_list = []

    while True: 
        image_row_list = []

        for _ in range(columns):
            if count < len(bounding_boxes):
                img = bounding_boxes[count]
                image_row_list.append(img)
                count += 1
            else: # Fill up the row with black images
                img = np.zeros((BOUNDING_BOX_IMAGE_SIZE, BOUNDING_BOX_IMAGE_SIZE, 3))
                img = np.asarray(img, dtype = np.uint8)
                image_row_list.append(img)
                count += 1
        
        if len(image_row_list) == 0:
            break
            
        image_row = image_row_list[0]

        for i in range(1, len(image_row_list)):
            image_row = cv2.hconcat([image_row, image_row_list[i]])
        
        image_list.append(image_row)

        if count >= len(bounding_boxes):
            break
        
        if len(image_list) >= rows:
            break

    all_bounding_boxes = image_list[0]

    for i in range(1, len(image_list)):
        all_bounding_boxes = cv2.vconcat((all_bounding_boxes, image_list[i]))

    labeling = {}

    while True:
        bounding_boxes_img = all_bounding_boxes.copy()
        bounding_box_select_x = (mouse_x // BOUNDING_BOX_IMAGE_SIZE) * BOUNDING_BOX_IMAGE_SIZE
        bounding_box_select_y = (mouse_y // BOUNDING_BOX_IMAGE_SIZE) * BOUNDING_BOX_IMAGE_SIZE
        cv2.rectangle(bounding_boxes_img, (bounding_box_select_x, bounding_box_select_y), 
                        (bounding_box_select_x + BOUNDING_BOX_IMAGE_SIZE, bounding_box_select_y + BOUNDING_BOX_IMAGE_SIZE),
                        (0, 255, 0), 1)
        
        font = cv2.FONT_HERSHEY_SIMPLEX

        for key in labeling:
            x, y = key 
            cv2.putText(bounding_boxes_img, str(labeling[key]), (x, y), font, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        
        cv2.imshow("bounding boxes", bounding_boxes_img)
        key = cv2.waitKey(1) & 0xFF

        x = (mouse_x // BOUNDING_BOX_IMAGE_SIZE) * BOUNDING_BOX_IMAGE_SIZE + BOUNDING_BOX_IMAGE_SIZE // 2
        y = (mouse_y // BOUNDING_BOX_IMAGE_SIZE) * BOUNDING_BOX_IMAGE_SIZE + BOUNDING_BOX_IMAGE_SIZE // 2

        # You can change the keys used for labeling if you want..
        if key == ord('1'):
            labeling[(x, y)] = 1
        
        if key == ord('2'):
            labeling[(x, y)] = 2
        
        if key == ord('3'):
            labeling[(x, y)] = 3
        
        if key == ord('4'):
            labeling[(x, y)] = 4

        if key == ord('5'):
            labeling[(x, y)] = 5

        if key == ord('6'):
            labeling[(x, y)] = 6

        if key == ord('7'):
            labeling[(x, y)] = 7

        if key == ord('8'):
            labeling[(x, y)] = 8
        
        if key == ord('9'):
            labeling[(x, y)] = 9

        # Remove the current label
        if key == ord('r'):
            labeling.pop((x, y), None)

        # Quits and doesn't save 
        if key == ord('q'):
            exit()
        
        # Save the annotation and go to the next image
        if key == ord(' '):
            break

    for key in labeling:
        label = [0] * NR_OF_CLASSES
        try:
            label[labeling[key] - 1] = 1
        except:
            continue

        x, y = key
        x -= BOUNDING_BOX_IMAGE_SIZE//2
        y -= BOUNDING_BOX_IMAGE_SIZE//2
        
        img = all_bounding_boxes[y: y + BOUNDING_BOX_IMAGE_SIZE, x : x + BOUNDING_BOX_IMAGE_SIZE]

        # Add cropped image to dataset
        data_x.append(img)
        # Add label to dataset
        data_y.append(label)
    
if not os.path.exists(path): # If /home/username/data does not exists, create it (although is should already be there)
    os.mkdir(path)

# Convert list to numpy array
data_x = np.asarray(data_x)
data_y = np.asarray(data_y)

# Save the two numpy files
np.save(os.path.join(path, "train_classifier.npy"), data_x)
np.save(os.path.join(path, "label_classifier.npy"), data_y)
