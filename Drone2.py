import ps_drone
import time

drone = ps_drone.Drone()
drone.startup()
drone.changeIP("192.168.1.102")
time.sleep(5)
print "Done"
