import cv2
import numpy as np
import time

startTime = time.time()
###Define boundaries of colours to detect, [([b,g,r])]
boundaries = [
([17, 91, 139], [92, 170, 190]) #Yellow
]

img = cv2.imread('Ball/18.jpg',1)
pImg = cv2.resize(img,(800,600))		# Resize

for (lower, upper) in boundaries:
    #Create numpy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")

    #Find colours within boundaries and apply mask
    mask = cv2.inRange(pImg, lower, upper)
    output = cv2.bitwise_and(pImg, pImg, mask = mask)


ret,thresh = cv2.threshold(mask, 40, 255, 0)
im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

if len(contours) != 0:
    #Draw in blue the found contours
    cv2.drawContours(output, contours, -1, 255, 3)

    #Find the largest contour
    c = max(contours, key = cv2.contourArea)
    #print str(cv2.contourArea(c))

    #Contour must be this big to count as ball. If number too small when no ball present may detect anything
    if cv2.contourArea(c) > 5250:
        #Draw contour with circle
        (x,y),radius = cv2.minEnclosingCircle(c)
        center = (int(x),int(y))
        diameter = radius * 2
        radius = int(radius)
        cv2.circle(output,center,radius,(0,255,0),2)
        #Draw contour with a rectangle
        '''
        x, y, w, h = cv2.boundingRect(c)
        #Draw contour in green
        cv2.rectangle(output, (x,y),(x+w, y+h), (0, 255, 0),2)
        '''

#Show the images
endTime = time.Time()
print "Time taken: " + str(endTime- startTime)
cv2.imshow("Ball", np.hstack([pImg, output]))
cv2.waitKey(0)							# OpenCV for Linux has a bug and needs this line
cv2.destroyAllWindows()
