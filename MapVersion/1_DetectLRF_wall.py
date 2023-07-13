from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math
import random

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 32

#enable distance sensors
fds = robot.getDevice('frontDist')
lds = robot.getDevice('leftDist')
rds = robot.getDevice('rightDist')
fds.enable(timestep)
lds.enable(timestep)
rds.enable(timestep)

# getting the position sensors
lps = robot.getDevice('left wheel sensor')
rps = robot.getDevice('right wheel sensor')
lps.enable(timestep)
rps.enable(timestep)

# enable camera and recognition
camera = robot.getDevice('camera')
camera.enable(timestep)

left_dist = lds.getValue() 
right_dist = rds.getValue() 
front_dist = fds.getValue() 

def getSurrounding():
    if lds.getValue() < 0.3 :
        #print(f'Left Wall Detected: {lds.getValue()}')
        print(f'Left Wall Detected')
    if rds.getValue() < 0.3 :
        #print(f'Right Wall Detected: {rds.getValue()}') 
        print(f'Right Wall Detected') 
    if fds.getValue() < 0.3 :
        #print(f'Front Wall Detected: {fds.getValue()}') 
        print(f'Front Wall Detected')

while robot.step(timestep) != -1:
    getSurrounding()