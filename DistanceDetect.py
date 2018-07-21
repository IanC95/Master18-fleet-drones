import cv2
import numpy as np



pImg = cv2.imread('Ball/17-5m.jpg',1)
#pImg = cv2.resize(img,(800,600))		# Resize

#Convert image from B,G,R to HSV
hsv = cv2.cvtColor(pImg, cv2.COLOR_BGR2HSV)

#range of orange in HSV
lower_yellow = np.array([20, 100, 100])   #Gimp values (37, 60, 54)
upper_yellow = np.array([30, 255, 255]) #Gimp values (51, 51, 80)

#Threshold the hsv image
mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

#Bitwise-AND the mask and original image
output = cv2.bitwise_and(pImg, pImg, mask = mask)

ret,thresh = cv2.threshold(mask, 40, 255, 0)
im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

if len(contours) != 0:
    #Draw in blue the found contours
    cv2.drawContours(output, contours, -1, 255, 3)

    #Find the largest contour
    c = max(contours, key = cv2.contourArea)
    print str(cv2.contourArea(c))

    #Contour must be this big to count as ball. If number too small when no ball present may detect anything
    if cv2.contourArea(c) > 500:
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
        P is width in pixels of object, diameter in calculations from above, in my test photo 638.691894531 was calculated
        #print str(diameter)
        D is distance to object, 50cm in test
        W is width of object in real life, 20cm for ball
        F = (638.691894531 * 50)/20  = 1596.729736

        D' = (W*F)/P
        '''
        #print str(diameter)
        W = 20
        F = 1596.729736
        P = diameter
        Distance = (W*F)/(P*1.0) #*1.0 to ensure float calculates properly
        #print Distance
        text = "Estimated distance = " + str(round(Distance, 1)) + " centimeters"
        font = cv2.FONT_HERSHEY_SIMPLEX
        pImg = cv2.resize(pImg,(800,600))		# Resize
        output = cv2.resize(output,(800,600))		# Resize
        cv2.putText(output, text,(100,100), font, 0.5,(255,255,255),1,cv2.LINE_AA)

#Show the images
pImg = cv2.resize(pImg,(800,600))		# Resize
output = cv2.resize(output,(800,600))		# Resize
cv2.imshow("Ball", np.hstack([pImg, output]))
cv2.waitKey(0)							# OpenCV for Linux has a bug and needs this line
cv2.destroyAllWindows()
