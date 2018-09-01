import time, sys
import ps_drone

drone = ps_drone.Drone()
drone.startup()

drone.reset()

while(drone.getBattery()[0]==-1):   time.sleep(0.1)
print "Battery: " + str(drone.getBattery()[0])+"% " + str(drone.getBattery()[1])
drone.useDemoMode(True)
time.sleep(0.5)

for i in drone.ConfigData:  print i

print "\nSample: "
for i in drone.ConfigData:
    if i[0] == "control:altitude_min":
        print str(i) + "Count: " + str(drone.ConfigDataCount) + " Timestamp: " + str(drone.ConfigDataTimeStamp)


print "----"
print "Setting \"control:altitude_min\" to \"20\"..."
CDC = drone.ConfigDataCount
NDC = drone.NavDataCount
refTime = time.time()
drone.setConfig("control:altitude_min", "20")
while CDC == drone.ConfigDataCount: time.sleep(0.001)
print "Finished after " + str(time.time()-refTime) + " seconds, " + str(drone.NavDataCount -NDC) + " NavData were recieved meanwhile."

for i in drone.ConfigData:
    if i[0] == "control:altitude_min":
        print str(i) + "Count: " + str(drone.ConfigDataCount) + " Timestamp: " + str(drone.ConfigDataTimeStamp)


stop = False
while not stop:
    drone.getConfig()
    key = drone.getKey()
    if key=="a":
         for i in drone.ConfigData:
             if i[0] == "control:altitude_min":
                 print str(i) + "Count: " + str(drone.ConfigDataCount) + " Timestamp: " + str(drone.ConfigDataTimeStamp)
    elif key:
        stop = True
