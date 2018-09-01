import cv2
import numpy as np



img = cv2.imread('Ball/Comparison.jpg',1)
pImg = cv2.resize(img,(720,360))		# Resize

#Convert image from B,G,R to HSV
hsv = cv2.cvtColor(pImg, cv2.COLOR_BGR2HSV)

boundaries = [
([17, 91, 139], [80, 170, 190]) #Yellow
]

#range of orange in HSV0
lower_orange = np.array([15, 95, 95])   #Gimp values (37, 60, 54)
upper_orange = np.array([35, 230, 230]) #Gimp values (51, 51, 80)

for (lower, upper) in boundaries:
    #Create numpy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")

    #Find colours within boundaries and apply mask
    mask = cv2.inRange(pImg, lower, upper)
    outputRGB = cv2.bitwise_and(pImg, pImg, mask = mask)


ret,thresh = cv2.threshold(mask, 40, 255, 0)
im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

if len(contours) != 0:
    #Draw in blue the found contours
    cv2.drawContours(outputRGB, contours, -1, 255, 3)

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
        cv2.circle(outputRGB,center,radius,(0,255,0),2)
        #Draw contour with a rectangle
        '''
        x, y, w, h = cv2.boundingRect(c)
        #Draw contour in green
        cv2.rectangle(output, (x,y),(x+w, y+h), (0, 255, 0),2)
        '''

#Threshold the hsv image
mask = cv2.inRange(hsv, lower_orange, upper_orange)

#Bitwise-AND the mask and original image
outputHSV = cv2.bitwise_and(pImg, pImg, mask = mask)

ret,thresh = cv2.threshold(mask, 40, 255, 0)
im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

if len(contours) != 0:
    #Draw in blue the found contours
    cv2.drawContours(outputHSV, contours, -1, 255, 3)

    #Find the largest contour
    c = max(contours, key = cv2.contourArea)
    #print str(cv2.contourArea(c))

    #Contour must be this big to count as ball. If number too small when no ball present may detect anything
    if cv2.contourArea(c) > 525:
        #Draw contour with circle
        (x,y),radius = cv2.minEnclosingCircle(c)
        center = (int(x),int(y))
        diameter = radius * 2
        radius = int(radius)
        cv2.circle(outputHSV,center,radius,(0,255,0),2)
        #Draw contour with a rectangle
        '''
        x, y, w, h = cv2.boundingRect(c)
        #Draw contour in green
        cv2.rectangle(output, (x,y),(x+w, y+h), (0, 255, 0),2)
        '''

#Show the images
cv2.imshow("Ball", np.vstack([np.hstack([pImg, outputRGB]), np.hstack([pImg, outputHSV])]))
cv2.waitKey(0)							# OpenCV for Linux has a bug and needs this line
cv2.destroyAllWindows()
