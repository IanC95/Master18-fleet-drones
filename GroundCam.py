import time, sys
import ps_drone

drone = ps_drone.Drone()
drone.startup()

drone.reset()
while (drone.getBattery()[0] == -1): time.sleep(0.1)
drone.useDemoMode(True)
drone.getNDpackage(["demo, vision_detect"])
time.sleep(0.5)

CDC = drone.ConfigDataCount
print drone.Version
drone.setConfigAllID()
drone.sdVideo()
drone.groundCam(True)
while CDC == drone.ConfigDataCount: time.sleep(0.001)

drone.startVideo()
drone.showVideo()
for i in drone.ConfigData:
    if i[0] == "video:video_channel":
        print str(i[1])

print "<Space> to toggle Cam"
IMC = drone.VideoImageCount
stop = False
ground = False
while not stop:
    while drone.VideoImageCount == IMC: time.sleep(0.01)
    IMC = drone.VideoImageCount
    drone.getConfig
    key = drone.getKey()
    if key=="a":
        if ground: ground = False
        else:   ground = True
        for i in drone.ConfigData:
            if i[0] == "video:video_channel":
                print str(i[1])
        drone.groundVideo(ground)
    elif key and key != "a": stop = True
