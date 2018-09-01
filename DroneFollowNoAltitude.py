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


if drone.State[31] == 1:
	drone.printYellow("Drone is in emergency mode, attempting reset.")
	drone.reset()
	time.sleep(2)
	drone.State[31] = 1
	time.sleep(2)
	#sys.exit()

if drone.State[31] == 1:
	drone.printRed("Reset failed, please remove battery and reboot drone.")
	sys.exit()

##### Mainprogram begin #####
drone.setConfigAllID()				# Go to multiconfiguration-mode
drone.sdVideo()						# Choose lower resolution (hdVideo() for...well, guess it) If HD used camera must be recallibrated
drone.frontCam()					# Choose front view
CDC = drone.ConfigDataCount
while CDC == drone.ConfigDataCount:	time.sleep(0.0001)	# Wait until it is done (after resync is done)
drone.startVideo()					# Start video-function
drone.setSpeed(0.3)

###Prepare for takeoff
drone.trim()
print("Calibrating trim")
drone.getSelfRotation(5)
print "Auto=alternation:	" + str(drone.selfRotation)+"deg/sec"

#range of orange in HSV
lower_yellow = np.array([15, 95, 95])   #Gimp values (37, 60, 54)
upper_yellow = np.array([35, 230, 230]) #Gimp values (51, 51, 80)

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

def distance(focal, itemWidth, pixelWidth):
	'''
	I need the focal lenght of the camera, focal
	Can be calculated focal = (pixelWidth*Distance)/itemWidth
	pixelWidth is width in pixels of object, in my test photo 350.108642578 was calculated
	Distance is distance to object, 30cm in test
	itemWidth is width of object in real life, 20cm for ball
	focal = (350.108642578 * 30)/20  = 525.162963867

	distance = (itemWidth*focal)/pixelWidth
	'''
	distance = (itemWidth*focal)/(pixelWidth*1.0) #*1.0 to ensure float calculates properly
	return distance

def detectColor(inputImage, lowerBound, upperBound):
	#Returns an image that has been thresholded between the lower and upper bounds, the mask applied to the original image to make that image and all of the contours present in the new image
	#Convert image from B,G,R to HSV
	hsv = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HSV)
	#Threshold the hsv image
	mask = cv2.inRange(hsv, lowerBound, upperBound)

	#Bitwise-AND the mask and original image
	outputImage = cv2.bitwise_and(inputImage, inputImage, mask = mask)

	ret,thresh = cv2.threshold(mask, 40, 255, 0)
	im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

	return outputImage, mask, contours

for i in drone.ConfigData:
    if i[0] == "control:altitude_min":
        print str(i) + "Count: " + str(drone.ConfigDataCount) + " Timestamp: " + str(drone.ConfigDataTimeStamp)

print "Setting \"control:altitude_min\" to \"20\"..."
CDC = drone.ConfigDataCount
NDC = drone.NavDataCount
refTime = time.time()
drone.setConfig("control:altitude_min", "20")
while CDC == drone.ConfigDataCount: time.sleep(0.001)
print "Finished after " + str(time.time()-refTime) + " seconds, " + str(drone.NavDataCount -NDC) + " NavData were recieved meanwhile."

drone.getConfig()
for i in drone.ConfigData:
    if i[0] == "control:altitude_min":
        #print str(i) + "Count: " + str(drone.ConfigDataCount) + " Timestamp: " + str(drone.ConfigDataTimeStamp)
		if i[1] != "20":
			drone.printYellow("Minimum altitude set to: " + str(i[1]) + " recommended value is 20.")


#Takeoff then wait for calibration
drone.takeoff()
#While drone state is "landed" wait
while drone.NavData["demo"][0][2]:	time.sleep(0.1)

print"Drone is flying"
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
	Distance = 150


	output, mask, contours = detectColor(pImg, lower_yellow, upper_yellow)

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

			F = 525.162963867
			W = 20

			Distance = distance(F, W, diameter)

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

	#print drone.NavData["demo"][3]

	UpDown = 0
	turnLeftRight = 0
	backForward = 0
	#Using prediction of where ball will be, rotate to face ball
	if predCenter[0] > 280 and predCenter[0] < 360:
		if not Halted:
			drone.stop()
			Halted = True
		#If facing ball, try to get distance between 0.5m and 1.5m
		if Distance < 150 and Distance > 50:
			drone.stop()
		elif Distance  > 150:
			backForward = 0.1
			#drone.moveForward(0.1)
		elif Distance < 50:
			backForward = -0.1
			#drone.moveBackward(0.1)
		#print "Current: " + str(center) + " Predicted: " + str(predCenter) + " Holding"
	elif predCenter[0] < 280:
		turnLeftRight = -0.4
		#drone.turnLeft(0.3)
		Halted = False
		#print "Current: " + str(center) + " Predicted: " + str(predCenter) + " Turn Left"
	elif predCenter[0] > 360:
		turnLeftRight = 0.4
		#drone.turnRight(0.3)
		Halted = False
		#print "Current: " + str(center) + " Predicted: " + str(predCenter) + " Turn Right"
	prevCenter = center

	'''
	if drone.NavData["demo"][3] > 85:
		UpDown = -0.5
		#drone.moveDown(0.5)
	else:
		UpDown = 0
		#drone.moveUp(0)
	'''

	drone.move(0.0, backForward, UpDown, turnLeftRight)
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
