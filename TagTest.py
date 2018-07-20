##Drone Start##
import time, sys
import ps_drone

drone = ps_drone.Drone()
drone.startup()

drone.reset()
while (drone.getBattery()[0]==-1):	time.sleep(0.1)	#Reset completed?
print "Battery: "+str(drone.getBattery()[0])+"%" +str(drone.getBattery()[1])
drone.useDemoMode(True)	#15 basic datasets/sec
drone.getNDpackage(["demo", "vision_detect"])	#Packets to be decoded
time.sleep(0.5)	#give drone time to wake properly

###Main Program###
NDC = drone.NavDataCount
#Setting up detection
drone.setConfig("detect:detect_type", "3")	#Universal detection
drone.setConfig("detect:detections_select_h", "0")	#Front: None
drone.setConfig("detect:detections_select_v", "128")	#Ground: Oriented Roundel
CDC = drone.ConfigDataCount
while CDC == drone.ConfigDataCount:	time.sleep(0.001)	#Wait until sync is done

#Get detections
end = False
while not end:
	while drone.NavDataCount == NDC:	time.sleep(0.001)	#wait for new navdata
	if drone.getKey():	end = True
	tagNum = drone.NavData["vision_detect"][0]	#No. found tags
	tagX	= drone.NavData["vision_detect"][2] #Horizontal pos
	tagY	= drone.NavData["vision_detect"][3] #Vertical pos
	tagZ	= drone.NavData["vision_detect"][6] #Distance
	tagRot	= drone.NavData["vision_detect"][7] #Rotation

	#Show detects
	if tagNum:
		for i in range(0,tagNum):
			print "Tag no. " + str(i)+" : X = "+str(tagX[i]) + " Y = " +str(tagY[i])\
			+"	Dist = " + str(tagZ[i])+ " Orientation= " + str(tagRot[i])

	NDC = drone.NavDataCount