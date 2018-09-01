import ps_drone

drone1 = ps_drone.Drone()
drone2 = ps_drone.Drone()
print "1"

drone1.DroneIP = "192.168.1.101"
drone2.DroneIP = "192.168.1.102"
print "2"

drone1.startup()
print "3"
drone2.startup()
print "4"

drone1.reset()
drone2.reset()
print "5"

while (drone1.getBattery()[0]==-1):	time.sleep(0.1)	#Reset completed?
while (drone2.getBattery()[0]==-1):	time.sleep(0.1)	#Reset completed?
print "6"

print "Drone1 battery: "+str(drone1.getBattery()[0])+"% " +str(drone1.getBattery()[1])
print "Drone2 battery: "+str(drone2.getBattery()[0])+"%" +str(drone2.getBattery()[1])
print "7"
