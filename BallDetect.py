import cv2
import numpy as np

###Define boundaries of colours to detect, [([b,g,r])]
boundaries = [
([20, 30, 100], [95, 150, 255]) #Orange
]

img = cv2.imread('Ball/1.jpg',1)
pImg = cv2.resize(img,(800,600))		# Resize

for (lower, upper) in boundaries:
    #Create numpy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")

    #Find colours within boundaries and apply mask
    mask = cv2.inRange(pImg, lower, upper)
    output = cv2.bitwise_and(pImg, pImg, mask = mask)

#Show the images
cv2.imshow("Ball", np.hstack([pImg, output]))
cv2.waitKey(0)							# OpenCV for Linux has a bug and needs this line
cv2.destroyAllWindows()
