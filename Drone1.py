import ps_drone

drone = ps_drone.Drone()
drone.startup()
drone.changeIP("192.168.1.101")
print "Done"
