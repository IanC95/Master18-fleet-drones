import time
import ps_drone

drone = ps_drone.Drone()
drone.startup()

drone.takeoff()
time.sleep(7.5)

drone.moveForward()
time.sleep(1)
drone.stop()
time.sleep(2)

drone.moveBackward(0.25)
time.sleep(1.5)
drone.stop()
time.sleep(2)

drone.setSpeed(1.0)
print drone.setSpeed()

drone.turnLeft()
time.sleep(2)
drone.stop()
time.sleep(2)

drone.land()