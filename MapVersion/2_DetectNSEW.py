import time
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

# enable inertiaUnit
inertial_unit = robot.getInertialUnit("inertial unit")
inertial_unit.enable(timestep)

# enable camera
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


def GetRobotRotation():
    roll, pitch, yaw = inertial_unit.getRollPitchYaw()

    # Normalize yaw angle to be within 0-2pi range
    yaw = (yaw + 2 * 3.14159) % (2 * 3.14159)

    # Classify the yaw angle to cardinal direction
    if (yaw >= 0 and yaw < 3.14159 / 4) or (yaw >= 7 * 3.14159 / 4 and yaw < 2 * 3.14159):
        direction = "North"
    elif yaw >= 3.14159 / 4 and yaw < 3 * 3.14159 / 4:
        direction = "West"
    elif yaw >= 3 * 3.14159 / 4 and yaw < 5 * 3.14159 / 4:
        direction = "South"
    else:
        direction = "East"

    # Print the cardinal direction
    print("Facing direction:", direction)

while robot.step(timestep) != -1:
    getSurrounding()
    GetRobotRotation()