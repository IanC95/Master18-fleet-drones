##Drone Start##
import time, sys
import ps_drone

drone = ps_drone.Drone()
drone.startup()

drone2 = ps_drone.Drone()
drone2.startup()

drone.reset()
drone2.reset()

while (drone.getBattery()[0]==-1):	time.sleep(0.1)	#Reset completed?
while (drone2.getBattery()[0]==-1):	time.sleep(0.1)	#Reset completed?

print "Drone1 battery: "+str(drone.getBattery()[0])+"%" +str(drone.getBattery()[1])
print "Drone2 battery: "+str(drone2.getBattery()[0])+"%" +str(drone2.getBattery()[1])
drone.useDemoMode(True)	#15 basic datasets/sec
drone2.useDemoMode(True)	#15 basic datasets/sec
drone.getNDpackage(["demo"])	#Packets to be decoded
drone2.getNDpackage(["demo"])	#Packets to be decoded
time.sleep(0.5)	#give drone time to wake properly

###Main Program###
#List the config
#for i in drone.ConfigData:	print i

#Closer look at an option
for i in drone.ConfigData:
	if i[0]=="general:drone_serial":
		print str(i)+" Count: " + str(drone.ConfigDataCount) + " Timestamp: " + str(drone.ConfigDataTimeStamp)

for i in drone2.ConfigData:
	if i[0]=="general:drone_serial":
		print str(i)+" Count: " + str(drone2.ConfigDataCount) + " Timestamp: " + str(drone2.ConfigDataTimeStamp)
'''
#And change it
print"-----"
print "Setting \"control:altitude_max\" to \"2999\"..."
CDC = drone.ConfigDataCount
NDC = drone.NavDataCount
refTime = time.time()
drone.setConfig("control:altitude_max", "2999")	#Change the option
while CDC == drone.ConfigDataCount: time.sleep(0.001)	#Wait till done
print "Finished after " + str(time.time()-refTime)+" seconds, " + str(drone.NavDataCount-NDC) + "NavData were recieved meanwhile."

for i in drone.ConfigData:
	if i[0]=="control:altitude_max":
		print str(i) + "Count: " + str(drone.ConfigDataCount)\
		+ " Timestamp: " + str(drone.ConfigDataTimeStamp)


CDC = drone.ConfigDataCount
NDC = drone.NavDataCount
refTime = time.time()
drone.getConfig()
while CDC == drone.ConfigDataCount:	time.sleep(0.001)
'''
