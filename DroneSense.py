##Drone Start##
import time, sys
import ps_drone

drone = ps_drone.Drone()
drone.startup()

drone.reset()
while (drone.getBattery()[0]==-1):	time.sleep(0.1)	#Reset completed?
print "Battery: "+str(drone.getBattery()[0])+"%" +str(drone.getBattery()[1])
drone.useDemoMode(True)	#15 basic datasets/sec
drone.getNDpackage(["demo"])	#Packets to be decoded
time.sleep(0.5)	#give drone time to wake properly

###Main Program###
NDC = drone.NavDataCount
state = 0
end = False

while not end:
	while drone.NavDataCount==NDC:	time.sleep(0.001)	#Wait for new navdata
	if drone.getKey():
		state+=1
		if state==1: drone.addNDpackage([¨demo¨]) # Add ¨demo¨ to list
		if state==2: drone.getNDpackage([¨all¨]) # Get these packages
		if state==3: drone.delNDpackage([¨all¨]) # Clear decoder-list
		if state==3: drone.useDemoMode(False) # switch to full-mode
		if state==4: drone.addNDpackage([¨demo¨])
		if state==5: drone.getNDpackage([¨all¨])
		if state>5: sys.exit()
	print "------------------"
	if state==0: print¨##### Demo Mode: On NavData-Packages: None¨
	if state==1: print¨##### Demo Mode: On NavData-Packages: Demo¨
	if state==2: print¨##### Demo Mode: On NavData-Packages: All¨
	if state==3: print¨##### Demo Mode: Off NavData-Packages: None¨
	if state==4: print¨##### Demo Mode: Off NavData-Packages: Demo¨
	if state==5: print¨##### Demo Mode: Off NavData-Packages: All¨
	if drone.NavDataCount-NDC>1:
		print¨Lost ¨+str(drone.NavDataCount-NDC-1)+¨ NavData¨
	print¨Number of package : ¨+str(drone.NavDataCount)
	print¨Receifetime : ¨+str(drone.NavDataTimeStamp)+¨ , \t¨\
	+str(time.time()-drone.NavDataTimeStamp)+¨ sec ago¨
	print¨Time to decode : ¨+str(drone.NavDataDecodingTime)
	print¨Included packages : ¨+str(drone.NavData.keys())
	print¨State-data : ¨+str(drone.State)
	try: print¨Demo-data : ¨+str(drone.NavData["demo"])
	except: pass
	NDC = drone.NavDataCount
