##### Suggested clean drone startup sequence #####
import time, sys
import ps_drone													# Import PS-Drone-API
#import cv2														# Import OpenCV

drone = ps_drone.Drone()										# Start using drone
drone.startup()													# Connects to drone and starts subprocesses

drone.reset()													# Sets drone's status to good (LEDs turn green when red)
while (drone.getBattery()[0]==-1):	time.sleep(0.1)				# Waits until drone has done its reset
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-status
drone.useDemoMode(False)
drone.getNDpackage(["all"])										# Just give me 15 basic dataset per second (is default anyway)
time.sleep(0.5)

##Main Program##
CDC = drone.ConfigDataCount
NDC = drone.NavDataCount
lat = 0
long = 0
alt = 0

end = False

while not end:
    while NDC == drone.NavDataCount: time.sleep(0.001) #wait for new data
    if drone.getKey():  end = True

    drone.getConfig()
    for i in drone.ConfigData:
        if i[0] == "gps:latitude":
            lat = i[1]
        if i[0] == "gps:longitude":
            long = i[1]
    print "Latitude: " + str(lat) + " Longitude: " + str(long)

    NDC = drone.NavDataCount
    CDC = drone.ConfigDataCount
