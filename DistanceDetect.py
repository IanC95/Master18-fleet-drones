import cv2
import numpy as np



img = cv2.imread('Ball/8.jpg',1)
pImg = cv2.resize(img,(800,600))		# Resize

#Convert image from B,G,R to HSV
hsv = cv2.cvtColor(pImg, cv2.COLOR_BGR2HSV)

#range of orange in HSV
lower_orange = np.array([7, 65, 65])
upper_orange = np.array([10, 255, 255])

#Threshold the hsv image
mask = cv2.inRange(hsv, lower_orange, upper_orange)

#Bitwise-AND the mask and original image
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
    if cv2.contourArea(c) > 300:
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
        #Distance Detection
        '''
        Ball is approx 2 inches in diameter
        I need the focal lenght of the camera, F
        Can be calculated F = (P*D)/W
        P is width in pixels of object, diameter in calculations from above, in my test photo 97.0234298706 was calculated
        #print str(diameter)
        D is distance to object, 12 inches in test
        W is width of object in real life, 2 inches for ball
        F = (97px*12in)/2in = 582.1405792

        D' = (W*F)/P
        '''
        #print str(diameter)
        W = 2
        F = 582.1405792
        P = diameter
        Distance = (W*F)/(P*1.0) #*1.0 to ensure float calculates properly
        #print Distance
        text = "Estimated distance = " + str(int(Distance)) + " inches"
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(output, text,(center[0]-100,center[1]-50), font, 0.5,(255,255,255),1,cv2.LINE_AA)

#Show the images
cv2.imshow("Ball", np.hstack([pImg, output]))
cv2.waitKey(0)							# OpenCV for Linux has a bug and needs this line
cv2.destroyAllWindows()
