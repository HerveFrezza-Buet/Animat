# Generates a colored image to see which Hue to select for a given RGB color

import cv2
import numpy as np

wscale = 2
width=180*wscale
height=50
image = np.zeros((height,width,3), np.uint8)
for i in range(height):
    for j in range(width):
        image[i,j,0] = j * 180/width
        image[i,j,1] = 255
        image[i,j,2] = 255

bgr_image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
for i in range(height):
    for j in range(width):
        if j != 0 and j % (30*wscale) == 0 :
            bgr_image[i,j,0] = 0
            bgr_image[i,j,1] = 0
            bgr_image[i,j,2] = 0
            
cv2.imwrite('mire.png', bgr_image)

