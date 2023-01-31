import cv2
import numpy as np 
import matplotlib.pyplot as plt


image = cv2.imread("test_image.jpg")

rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # Convert to RGB format
plt.imshow(rgb_image) # Use matplotlib to show image, to get coordinates
plt.show() # The program will halt here to show the image, when closed it will continue

# Source points are the points from the original image, they indicate the start and end position of a line
# You can determine the coordinates via the plot
source_points = np.float32([[50, 800], [350, 400], [750, 800], [450, 400]])

# Destination points are the points to which the lines should transform to,
# such that the lines are parallel (as if the camera was looking from above)
destination_points = np.float32([[350, 800], [350, 400], [450, 800], [450, 400]])

M = cv2.getPerspectiveTransform(source_points, destination_points)
height, width, _ = image.shape

warped_image = cv2.warpPerspective(image, M, (width, height))
cv2.imshow("warped image", warped_image)
cv2.waitKey(0)