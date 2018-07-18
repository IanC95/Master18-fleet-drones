##Drone Start##
import time, sys
import ps_drone

drone = ps_drone.Drone()
drone.startup()

drone.reset()
while (drone.getBattery()[0]==-1):	time.sleep(0.1)	#Reset completed?
print "Battery: "+str(drone.getBattery()[0])+"%" +str(drone.getBattery()[1])
if drone.getBattery()[1]=="empty":	sys.exit()	#If battery low, abort

drone.useDemoMode(True)	#15 datasets/sec
drone.getNDpackage(["demo", "vision_detect"])	#Packets to be decoded, only decode what is neccassary for increased performance
time.sleep(0.5)	#give drone time to wake properly

###Main program
CDC = drone.ConfigDataCount
drone.setConfigAllID()  #Go to multiconfig mode
drone.sdVideo() #Choose lower res
drone.frontCam()    #Choose front view
while CDC == drone.ConfigDataCount: time.sleep(0.001) #wait until config done
drone.startVideo()  #Start video function
drone.showVideo()   #Display the video

print "<space> to toggle camera, any other key to stop"
IMC = drone.VideoImageCount #Num. endoded videoframes
stop = False
while not stop:
    while drone.VideoImageCount == IMC: time.sleep(0.01) #wait for new videoframes
    IMC = drone.VideoImageCount
    key = drone.getKey()
    if key:    stop = True
