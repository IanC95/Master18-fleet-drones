##Drone Start##
import time, sys
import ps_drone

drone = ps_drone.Drone()
drone.startup()

drone.reset()
while (drone.getBattery()[0]==-1):	time.sleep(0.1)	#Reset completed?
print "Battery: "+str(drone.getBattery()[0])+"%" +str(drone.getBattery()[1])
if drone.getBattery()[1]=="empty":	sys.exit()	#If battery low, abort

drone.useDemoMode(False)	#200 datasets/sec
drone.getNDpackage(["demo", "vision_detect", "altitude"])	#Packets to be decoded, only decode what is neccassary for increased performance
time.sleep(0.5)	#give drone time to wake properly

###Main Program###
NDC = drone.NavDataCount
#Setting up detection
drone.setConfig("detect:detect_type", "3")	#Universal detection
drone.setConfig("detect:detections_select_h", "0")	#Front: None
drone.setConfig("detect:detections_select_v", "128")	#Ground: Oriented Roundel
CDC = drone.ConfigDataCount
while CDC == drone.ConfigDataCount:	time.sleep(0.001)	#Wait until sync is done

###Prepare for takeoff
drone.trim()
print("Calibrating trim")
drone.getSelfRotation(5)
print "Auto=alternation:	" + str(drone.selfRotation)+"deg/sec"

#Takeoff then wait for calibration
drone.takeoff()
while drone.NavData["demo"][0][2]:	time.sleep(0.1)

print("Drone is flying")

#In flight routine
end = False
while not end:
	while drone.NavDataCount == NDC:	time.sleep(0.001)	#wait for new navdata
	if drone.getKey():	end = True
	tagNum = drone.NavData["vision_detect"][0]	#No. found tags
	tagX	= drone.NavData["vision_detect"][2] #Horizontal pos
	tagY	= drone.NavData["vision_detect"][3] #Vertical pos
	tagZ	= drone.NavData["vision_detect"][6] #Distance
	tagRot	= drone.NavData["vision_detect"][7] #Rotation
	height = drone.NavData["altitude"][3] #Estimated Altitude

	#Show detects
	if tagNum:
		for i in range(0,tagNum):
			print "Tag no. " + str(i)+" : X = "+str(tagX[i]) + " Y = " +str(tagY[i])\
			+"	Dist = " + str(tagZ[i])+ " Orientation= " + str(tagRot[i])
			
			'''
			###Commands for hovering above given target
			###Orientation
			##If 0 - 180, turn left
			##if 180-360, turn right
			RorL = 0	#Turn left or right?
			if tagRot[i] <= 180:
				RorL = -0.01 * (tagRot[i]/180.0)
			else:
				RorL = 0.01 * ((tagRot[i]-180)/180.0)
			turnLeftRight = RorL

			##Forward backward
			#0-500 = move back
			#500-1000 = move forward

			ForB = 0	#Forward or Backward?
			if tagY[i] <= 499:
				ForB = -1 * 0.001
			elif tagY[i] >= 501:
				ForB = 1 * 0.001
			else:
				ForB = 0
			backForward = ForB

			##Leftright
			#0-500 = move left
			#500-1000 = move right

			Leftright = 0	#Left or Right?
			if tagX[i] <= 499:
				Leftright = -1 * 0.001
			elif tagX[i] >= 501:
				Leftright = 1 * 0.001
			else:
				Leftright = 0
			'''
			##Updown
			#ideal = 45
			##Lower = go up
			##Higher = go down
			ideal = 55	#distance to aim for
			if height<ideal:
				moveUp(0.01)
			elif height>ideal:
				moveDown(0.01)
			else:
				pass

			#drone.move(Leftright, backForward, 0, turnLeftRight)


	NDC = drone.NavDataCount

###Shutdown Sequence
#Stop Moving
drone.stop()
#Give drone time to stop
time.sleep(0.25)
#Safely land drone
drone.land()
#Terminate program safely
drone.shutdown()