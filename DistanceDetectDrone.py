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
lower_yellow = np.array([15, 95, 95])   #Gimp values (37, 60, 54)
upper_yellow = np.array([35, 255, 255]) #Gimp values (51, 51, 80)

#Pre Flight values
prevCenter = (0,0)
predCenter = (0,0)
deltaCenter = (0,0)
center = (0,0)
backForward = 0
leftRight = 0
UpDown = 0
turnLeftRight = 0
Halted = False


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
	    if cv2.contourArea(c) > 250:
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
	        P is width in pixels of object, diameter in calculations from above, in my test photo 350.108642578 was calculated
	        #print str(diameter)
	        D is distance to object, 30cm in test
	        W is width of object in real life, 20cm for ball
	        F = (350.108642578 * 30)/20  = 525.162963867

	        D' = (W*F)/P
	        '''
	        #print str(diameter)
	        W = 20
	        F = 525.162963867
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

	#Calculate change in center and use to predict where the ball will be
	deltaCenter = (center[0] - prevCenter[0], center[1] - prevCenter[1])
	predCenter = (center[0] + deltaCenter[0], center[1] + deltaCenter[1])

	'''
	print "Current: " + str(center)
	print "Prev: " + str(prevCenter)
	print "Delta: " + str(deltaCenter)
	'''
	#Using prediction of where ball will be, rotate to face ball
	if predCenter[0] > 280 and predCenter[0] < 360:
		if not Halted:
			drone.stop()
			Halted = True
		#If facing ball, try to get distance between 1m and 2m
		if Distance < 200 and Distance > 100:
			drone.stop()
		elif Distance  > 200:
			drone.moveForward(0.2)
		elif Distance < 100:
			drone.moveBackward(0.2)
		#print "Current: " + str(center) + " Predicted: " + str(predCenter) + " Holding"
	elif predCenter[0] < 280:
		drone.turnLeft(0.3)
		Halted = False
		#print "Current: " + str(center) + " Predicted: " + str(predCenter) + " Turn Left"
	elif predCenter[0] > 360:
		drone.turnRight(0.3)
		Halted = False
		#print "Current: " + str(center) + " Predicted: " + str(predCenter) + " Turn Right"
	#drone.stop()
	#print str(turnLeftRight)
	prevCenter = center


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
