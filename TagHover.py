##Drone Start##
import time, sys, math
import ps_drone

def CosClamped(n, L, U):
	#Input number, Lower bound, upper bound
	if n < L:
		return 1#Lower than lower bound, return 1
	elif n > U:
		return -1#Higher than upper bound, return -1
	elif n > ((U-L*1.0)/2)-((U-L)*0.1) and n < ((U-L*1.0)/2)+((U-L)*0.1):
		return 0
	else:
		percent = (n - L)*1.0/(U-L)#Percent between lower and upper bound, multiplied by 1.0 cos python is stupid
		cosvalue = math.radians(percent * 180)#Scale between 0 and pi rad
		value = math.cos(cosvalue)#Cos the value
		return value


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

#Get estimated altitude when landed
StartHeight = -9999
while StartHeight == -9999:
	StartHeight = drone.NavData["altitude"][3]
	time.sleep(0.001)

prevX = 500
prevY = 500
prevAltitude = StartHeight

#Takeoff then wait for calibration
drone.takeoff()
#While drone state is "landed" wait
while drone.NavData["demo"][0][2]:	time.sleep(0.1)

print("Drone is flying")

backForward = 0
leftRight = 0
UpDown = 0
turnLeftRight = 0

#In flight routine
end = False
while not end:
	while drone.NavDataCount == NDC:
		time.sleep(0.001)	#wait for new navdata
	if drone.getKey():	end = True
	tagNum = drone.NavData["vision_detect"][0]	#No. found tags
	tagX	= drone.NavData["vision_detect"][2] #Horizontal pos
	tagY	= drone.NavData["vision_detect"][3] #Vertical pos
	tagZ	= drone.NavData["vision_detect"][6] #Distance
	tagRot	= drone.NavData["vision_detect"][7] #Rotation
	height = drone.NavData["altitude"][3]- StartHeight #Estimated Altitude




	##Updown
	#print("\nHeight = " + str(height))
	#UpDown = CosClamped(height, 200, 1600)/4.0#Calculate speed to go up or down on a cos curve
	#print str(drone.NavData["vision"][10])
	#Show detects
	if tagNum:
		for i in range(0,tagNum):
			print "Tag no. " + str(i)+" : X = "+str(tagX[i]) + " Y = " +str(tagY[i])\
			+"	Dist = " + str(tagZ[i])+ " Orientation= " + str(tagRot[i])

			###Commands for hovering above given target
			'''
			###Orientation
			##If 0 - 180, turn left
			##if 180-360, turn right
			RorL = 0	#Turn left or right?
			if tagRot[i] <= 180:
				RorL = -Speed * (tagRot[i]/180.0)
			else:
				RorL = Speed * ((tagRot[i]-180)/180.0)
			turnLeftRight = RorL
			'''

			Speed = 0.01

			##Forward backward
			#0-500 = move back
			#500-1000 = move forward
			'''
			backForward = 0	#Forward or Backward?
			if tagY[i] <= 400:
				backForward = 1 * Speed
			elif tagY[i] >= 600:
				backForward = -1 * Speed
			else:
				backForward = 0
			'''
			#backForward = CosClamped(tagY[i], 0, 1000)/4.0


			##Leftright
			#0-500 = move left
			#500-1000 = move right
			'''
			leftRight = 0	#Left or Right?
			if tagX[i] <= 400:
				leftRight = 1 * Speed
			elif tagX[i] >= 600:
				leftRight = -1 * Speed
			else:
				leftRight = 0
			'''
			#leftRight = CosClamped(tagX[i], 0, 1000)/4.0

			if tagY[i] <= 450 and tagY[i] < prevY: #Currently drifting backward
				backForward = backForward + Speed
			elif tagY[i] >=550 and tagY[i] > prevY: #Currently drifting forward
				backForward = backForward - Speed
			elif tagY[i] >=550 and tagY[i] <= prevY: #Drifting towards middle:
				if backForward < 0:
					backForward = backForward + Speed/10.0	#reduce speed
			elif tagY[i] <=450 and tagY[i] >= prevY:#Drifting towards middle
				if backForward > 0:
					backForward = backForward - Speed/10.0	#reduce speed
			else:
				backForward = 0

			if tagX[i] <= 450 and tagX[i] < prevX: #Currently drifting right
				leftRight = leftRight - Speed
			elif tagX[i] >=550 and tagX[i] > prevX: #Currently drifting left
				leftRight = leftRight + Speed
			elif tagX[i] >=550 and tagX[i] <= prevX: #Drifting towards middle:
					if leftRight > 0:
						leftRight = leftRight - Speed/10.0	#reduce speed
			elif tagX[i] <=450 and tagX[i] >= prevX:#Drifting towards middle
					if leftRight < 0:
						leftRight = leftRight + Speed/10.0	#reduce speed
			else:
				leftRight = 0

			if leftRight > 0.06:
				leftRight = 0.06
			elif leftRight < -0.06:
				leftRight = -0.06

			if backForward > 0.06:
				backForward = 0.06
			elif backForward < -0.06:
				backForward = -0.06
			print "Moving " + str(backForward) + " backward/forward and " + str(leftRight) + " left/right"
			prevX = tagX[i]
			prevY = tagY[i]


	drone.move(leftRight, backForward, UpDown, turnLeftRight)

	prevAltitude = drone.NavData["altitude"][3]- StartHeight
	NDC = drone.NavDataCount

###Shutdown Sequence
#Stop Moving
drone.stop()
#Give drone time to stop
time.sleep(0.5)
#Safely land drone
drone.land()
#Wait until drone says it has landed or timeout
count = 0
while not drone.NavData["demo"][0][2] or count>50:
	time.sleep(0.1)
	count = count + 1
#Terminate program safely
drone.shutdown()
