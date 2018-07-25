#########
# useVideo.py
# This program is part of the online PS-Drone-API-tutorial on www.playsheep.de/drone.
# It shows the usage of the video-function of a Parrot AR.Drone 2.0 using the PS-Drone-API.
# The drone will stay on the ground.
# Dependencies: a POSIX OS, openCV2, PS-Drone-API 2.0 beta or higher.
# (w) J. Philipp de Graaff, www.playsheep.de, 2016
##########
# LICENCE:
#   Artistic License 2.0 as seen on http://opensource.org/licenses/artistic-license-2.0 (retrieved December 2014)
#   Visit www.playsheep.de/drone or see the PS-Drone-API-documentation for an abstract from the Artistic License 2.0.
###########

##### Suggested clean drone startup sequence #####
import time, sys
import ps_drone													# Import PS-Drone-API
import cv2														# Import OpenCV
import numpy as np												# Import numpy

print "Hold escape to exit program"
drone = ps_drone.Drone()										# Start using drone
drone.startup()													# Connects to drone and starts subprocesses

drone.reset()													# Sets drone's status to good (LEDs turn green when red)
while (drone.getBattery()[0]==-1):	time.sleep(0.1)				# Waits until drone has done its reset
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-status
if drone.getBattery()[1]=="empty":	sys.exit()	#If battery low, abort

drone.useDemoMode(True)											# Just give me 15 basic dataset per second (is default anyway)
drone.getNDpackage(["demo"])	#Packets to be decoded, only decode what is neccassary for increased performance
time.sleep(0.5)	#give drone time to wake properly

##### Mainprogram begin #####
drone.setConfigAllID()				# Go to multiconfiguration-mode
drone.sdVideo()						# Choose lower resolution (hdVideo() for...well, guess it) If HD used camera must be recallibrated
drone.frontCam()					# Choose front view
CDC = drone.ConfigDataCount
while CDC == drone.ConfigDataCount:	time.sleep(0.0001)	# Wait until it is done (after resync is done)
drone.startVideo()					# Start video-function

###Prepare for takeoff
drone.trim()
print("Calibrating trim")
drone.getSelfRotation(5)
print "Auto=alternation:	" + str(drone.selfRotation)+"deg/sec"

#range of orange in HSV
lower_yellow = np.array([20, 100, 100])   #Gimp values (37, 60, 54)
upper_yellow = np.array([30, 255, 255]) #Gimp values (51, 51, 80)

#Pre Flight values
center = (0,0)
backForward = 0
leftRight = 0
UpDown = 0
turnLeftRight = 0


#Takeoff then wait for calibration
drone.takeoff()
#While drone state is "landed" wait
while drone.NavData["demo"][0][2]:	time.sleep(0.1)

print("Drone is flying")
IMC = 	 drone.VideoImageCount		# Number of encoded videoframes
stop =	 False
while not stop:
	while drone.VideoImageCount==IMC: time.sleep(0.01)	# Wait until the next video-frame
	IMC = drone.VideoImageCount
	key = drone.getKey()
	if key:		stop = True
	k = cv2.waitKey(33)
	if k == 27:
		stop = True
	pImg  = drone.VideoImage					# Copy video-image
	#pImg = cv2.resize(img,(720,360))		# Process video-image
	Distance = 99999
	#Convert image from B,G,R to HSV
	hsv = cv2.cvtColor(pImg, cv2.COLOR_BGR2HSV)



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
	    #print str(cv2.contourArea(c))

	    #Contour must be this big to count as ball. If number too small when no ball present may detect anything
	    if cv2.contourArea(c) > 100:
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
	        P is width in pixels of object, diameter in calculations from above, in my test photo 358.190826416 was calculated
	        #print str(diameter)
	        D is distance to object, 29.7cm in test
	        W is width of object in real life, 20cm for ball
	        F = (358.190826416 * 29.7)/20  = 531.9133772277

	        D' = (W*F)/P
	        '''
	        #print str(diameter)
	        W = 20
	        F = 531.9133772277
	        P = diameter
	        Distance = (W*F)/(P*1.0) #*1.0 to ensure float calculates properly
	        #print Distance
	        text = "Estimated distance = " + str(round(Distance, 1)) + " centimeters"
	        font = cv2.FONT_HERSHEY_SIMPLEX
	        pImg = cv2.resize(pImg,(640,360))		# Resize
	        output = cv2.resize(output,(640,360))		# Resize
	        cv2.putText(output, text,(100,100), font, 0.5,(255,255,255),1,cv2.LINE_AA)
	else:
		 pImg = cv2.resize(pImg,(640,360))		# Resize
		 output = cv2.resize(output,(640,360))		# Resize

	#Show the images
	cv2.imshow("Detection Mask", np.hstack([pImg, output]))
	cv2.waitKey(1)							# OpenCV for Linux has a bug and needs this line

	if center[0] > 280 and center < 360:
		drone.stop()
	elif center[0] < 280:
		drone.turnRight(0.3)
	elif center[0] > 360:
		drone.turnLeft(0.3)

	#drone.stop()
	#print str(turnLeftRight)


###Shutdown Sequence
#Stop Moving
drone.stop()
#Give drone time to stop
time.sleep(0.5)
#Safely land drone
drone.land()
#Wait until drone says it has landed or timeout
count = 0
while not drone.NavData["demo"][0][2] or count>450:
	time.sleep(0.01)
	count = count + 1
#Terminate program safely
drone.shutdown()
