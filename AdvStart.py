import time, sys
import ps_drone

drone = ps_drone.Drone()
drone.startup()

drone.reset()

while (drone.getBattery()[0]==-1):	time.sleep(0.1)
print "Battery: " + str(drone.getBattery()[0])+"%"+str(drone.getBattery()[1])
if drone.getBattery()[1]=="empty":	sys.exit()

drone.useDemoMode(True)
drone.getNDpackage(["demo"])
time.sleep(0.5)

drone.trim()
print("Calibrating Trim")
time.sleep(2)
drone.getSelfRotation(5)
print "Auto-alternation:	"+str(drone.selfRotation)+"deg/sec"

####Main Program####
drone.takeoff()
while drone.NavData["demo"][0][2]:	time.sleep(0.1)

print("Drone is flying")

leftRight = 0
backForward = 0.1
downUp = 0
turnLeftRight = 0
drone.move(leftRight, backForward, downUp, turnLeftRight)

timeToFlight = 2.5
refTime = time.time()
end = False

while not end:
	if drone.getKey():	end=True
	if time.time()-refTime>=timeToFlight:	end = True

drone.stop()
print("Drone Stopping")
time.sleep(2)

print "Drone turning 180 degrees left"
drone.turnAngle(-180,1,1)

drone.move(leftRight, backForward, downUp, turnLeftRight)

timeToFlight = 2.5
refTime = time.time()
end = False

while not end:
	if drone.getKey():	end=True
	if time.time()-refTime>=timeToFlight:	end = True

drone.stop()
print("Drone Stopping")
time.sleep(2)

print "Drone turning 180 degrees left"
drone.turnAngle(-180,1,1)

drone.stop()
print "Drone stopping then landing"
time.sleep(2)
drone.land()