from controller import Robot, Motor, DistanceSensor, Camera, Emitter, GPS, CameraRecognitionObject, InertialUnit,\
    PositionSensor
import struct
import numpy as np
import cv2 as cv
import pytesseract
import os
import matplotlib.pyplot as plt
import time
import math
import random
import sys

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = 32

# enable distance sensors
fds = robot.getDevice('frontDist')
lds = robot.getDevice('leftDist')
rds = robot.getDevice('rightDist')
fds.enable(timestep)
lds.enable(timestep)
rds.enable(timestep)

# enable wheel sensors
lps = robot.getDevice('left wheel sensor')
rps = robot.getDevice('right wheel sensor')
lps.enable(timestep)
rps.enable(timestep)

# enable inertiaUnit
inertial_unit = robot.getDevice("inertial unit")
inertial_unit.enable(timestep)

# enable camera
camera = robot.getDevice('camera')
camera.enable(timestep)

# enable GPS sensors
pos_sensor = robot.getDevice('gps')
pos_sensor.enable(timestep)


# detect the wall from robot (left / right / front of robot)
def getSurrounding():
    if lds.getValue() < 0.3:
        # print(f'Left Wall Detected: {lds.getValue()}')
        print(f'Left Wall Detected')

    if rds.getValue() < 0.3:
        # print(f'Right Wall Detected: {rds.getValue()}')
        print(f'Right Wall Detected')

    if fds.getValue() < 0.3:
        # print(f'Front Wall Detected: {fds.getValue()}')
        print(f'Front Wall Detected')


# detect the angle where robot is facing
def GetRobotRotation():
    roll, pitch, yaw = inertial_unit.getRollPitchYaw()

    # Normalize yaw angle to be within 0-2pi range
    yaw = (yaw + 2 * 3.14159) % (2 * 3.14159)

    # Classify the yaw angle to cardinal direction
    if (yaw >= 0 and yaw < 3.14159 / 4) or (yaw >= 7 * 3.14159 / 4 and yaw < 2 * 3.14159):
        direction = "N"
    elif yaw >= 3.14159 / 4 and yaw < 3 * 3.14159 / 4:
        direction = "W"
    elif yaw >= 3 * 3.14159 / 4 and yaw < 5 * 3.14159 / 4:
        direction = "S"
    else:
        direction = "E"

    # Print the direction
    print("Facing direction:", direction)
    return direction


# Map details
tile_size = 0.12
MAZE_COLUMNS = 8
MAZE_ROWS = 8

# New empty list
visited_cells = []
already_visited = []


def create_list_of_markers(length):
    return [[0, 0, 0, 0, 0] for _ in range(length)]


grid = create_list_of_markers(MAZE_COLUMNS * MAZE_ROWS)


# Original Grid [Visited, North, East, South, West]
# grid = [[0, 1, 0, 0, 1], [0, 1, 0, 0, 0], [0, 1, 0, 0, 0],[0, 1, 0, 0, 0],[0, 1, 0, 0, 0],[0, 1, 0, 0, 0],[0, 1, 0, 0, 0],[0, 1, 1, 0, 0], #starting grid
#        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
#        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
#        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
#        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
#        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
#        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
#        [0, 0, 0, 1, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 1, 0]]

def printMaze(maze):
    print("__________________________________________________________________________________")
    for i in range(8):
        x = i * 8
        if (maze[x][0] == 0):
            v1 = "?"
        else:
            v1 = "V"
        if (maze[x + 1][0] == 0):
            v2 = "?"
        else:
            v2 = "V"
        if (maze[x + 2][0] == 0):
            v3 = "?"
        else:
            v3 = "V"
        if (maze[x + 3][0] == 0):
            v4 = "?"
        else:
            v4 = "V"
        if (maze[x + 4][0] == 0):
            v5 = "?"
        else:
            v5 = "V"
        if (maze[x + 5][0] == 0):
            v6 = "?"
        else:
            v6 = "V"
        if (maze[x + 6][0] == 0):
            v7 = "?"
        else:
            v7 = "V"
        if (maze[x + 7][0] == 0):
            v8 = "?"
        else:
            v8 = "V"
        print("|  " + str(maze[x][1]) + "\t  " + str(maze[x + 1][1]) + "\t  " + str(maze[x + 2][1]) + "\t  " + str(
            maze[x + 3][1]) + "\t  " + str(maze[x + 4][1]) + "\t  " + str(maze[x + 5][1]) + "\t  " + str(
            maze[x + 6][1]) + "\t  " + str(maze[x + 7][1]) + "    |")
        print("|" + str(maze[x][4]) + " " + v1 + " " + str(maze[x][2]) + "\t" + str(
            maze[x + 1][4]) + " " + v2 + " " + str(maze[x + 1][2])
              + "\t" + str(maze[x + 2][4]) + " " + v3 + " " + str(maze[x + 2][2])
              + "\t" + str(maze[x + 3][4]) + " " + v4 + " " + str(maze[x + 3][2])
              + "\t" + str(maze[x + 4][4]) + " " + v5 + " " + str(maze[x + 4][2])
              + "\t" + str(maze[x + 5][4]) + " " + v6 + " " + str(maze[x + 5][2])
              + "\t" + str(maze[x + 6][4]) + " " + v7 + " " + str(maze[x + 6][2])
              + "\t" + str(maze[x + 7][4]) + " " + v8 + " " + str(maze[x + 7][2]) + "  |")
        print("|  " + str(maze[x][3]) + "\t  " + str(maze[x + 1][3]) + "\t  " + str(maze[x + 2][3]) + "\t  " + str(
            maze[x + 3][3]) + "\t  " + str(maze[x + 4][3]) + "\t  " + str(maze[x + 5][3]) + "\t  " + str(
            maze[x + 6][3]) + "\t  " + str(maze[x + 7][3]) + "    |")
        if (i == 7):
            print("|__________________________________________________________________________________|\n")
        else:
            print("|                                                                                  |")


def getRobotPosition():
    position = pos_sensor.getValues()
    # Calculate the tile position based on the coordinates
    x_pos = round(position[0], 1)
    y_pos = round(position[2], 1)

    # Dividing according to the tile size of map (12cm)
    start_tile_x = round((x_pos / tile_size))
    start_tile_y = round((y_pos / tile_size))

    # x = -3 ; y = -4
    diff_x = 3
    diff_y = 4

    # Move the robot in the calculated direction
    new_x = start_tile_x + diff_x
    new_y = start_tile_y + diff_y

    print('Updated X Position:' + str(new_x))
    print('Updated Y Position:' + str(new_y))

    return new_x, new_y


def CheckWallPosition(direction, grid, current_tile_x, current_tile_y, front, left, right):
    grid_position = FindGrid(grid, current_tile_x, current_tile_y)
    if direction == 'N':
        grid_position[1] = front
        grid_position[4] = left
        grid_position[2] = right

    elif direction == 'E':
        grid_position[2] = front
        grid_position[1] = left
        grid_position[3] = right

    elif direction == 'S':
        grid_position[3] = front
        grid_position[2] = left
        grid_position[4] = right

    elif direction == 'W':
        grid_position[4] = front
        grid_position[3] = left
        grid_position[1] = right

    return


# Index value for grid
def setItem(a, c, r, value):
    a[r * MAZE_COLUMNS + c] = value
    print('This Value', value)


# Get corresponding of current grid
def FindGrid(a, c, r):
    return a[r * MAZE_COLUMNS + c]


# Check if tile is visited
def is_visited_tiles(current_tile_x, current_tile_y):
    grid_position = FindGrid(grid, current_tile_x, current_tile_y)
    if not grid_position[0] == 1:
        visited_cells.append((current_tile_x, current_tile_y))
        grid_position[0] = 1
        print('Tile Saved into List')
    else:
        print('Tile Visited.')


def UpdateMaze(current_tile_x, current_tile_y, L, F, R, direction):
    grid_list = FindGrid(grid, current_tile_x, current_tile_y)
    grid_list[0] = 1  # mark cell visited
    n = (current_tile_x, current_tile_y)

    if n not in already_visited:
        # mark internal walls based on sensor readings
        if direction == 'N':
            grid_list[4] = L  # LEFT sensor = WEST wall
            grid_list[1] = F  # FRONT sensor = NORTH wall
            grid_list[2] = R  # RIGHT sensor = WEST wall
        elif direction == 'E':
            grid_list[1] = L  # LEFT sensor = NORTH wall
            grid_list[2] = F  # FRONT sensor = EAST wall
            grid_list[3] = R  # RIGHT sensor = SOUTH wall
        elif direction == 'S':
            grid_list[2] = L  # LEFT sensor = EAST wall
            grid_list[3] = F  # FRONT sensor = SOUTH wall
            grid_list[4] = R  # RIGHT sensor = WEST wall
        elif direction == 'W':
            grid_list[3] = L  # LEFT sensor = SOUTH wall
            grid_list[4] = F  # FRONT sensor = WEST wall
            grid_list[1] = R  # RIGHT sensor = NORTH wall
        already_visited.append(n)


def ObstacleAvoidance():
    Frontsensor = fds.getValue()
    Leftsensor = lds.getValue()
    Rightsensor = rds.getValue()
    left = 1 if Leftsensor < 0.15 else 0
    front = 1 if Frontsensor < 0.15 else 0
    right = 1 if Rightsensor < 0.15 else 0
    # print('L(', L,') F(', F,') R(', R,')')

    return (front, left, right)

# for victim detection
# create detector
detector=cv.xfeatures2d.SIFT_create()

# create Matcher object
FLANN_INDEX_KDITREE=0
flannParam=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
flann=cv.FlannBasedMatcher(flannParam,{})

# create sample features
dir = 'C:\\Users\\ASUS\\OneDrive - Sheffield Hallam University\\erebus-v23.0.5\\player_controllers\\sample_imgs'
objs = []
sift_features = []
for filename in os.listdir(dir):
    obj = os.path.join(dir, filename)
    trainImg = cv.imread(obj)
    trainKP, trainDesc = detector.detectAndCompute(trainImg, None)
    objs.append(filename[0])
    sift_features.append((trainKP, trainDesc))
    print(f'{filename[0]}: {len(trainKP)} key points')

print(objs)

# Set minimum number of matches
MIN_MATCH_COUNT = 9

def turn_right():
    #set left wheel speed
    speeds[0] = 0.6 * max_velocity
    #set right wheel speed
    speeds[1] = -0.2 * max_velocity

def turn_left():
    #set left wheel speed
    speeds[0] = -0.2 * max_velocity
    #set right wheel speed
    speeds[1] = 0.6 * max_velocity

def spin():
    #set left wheel speed
    speeds[0] = 0.6 * max_velocity
    #set right wheel speed
    speeds[1] = -0.6 * max_velocity

def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timeStep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms: # If time elapsed (converted into ms) is greater than value passed in
            break

def getColor():
    img = colorSensor.getImage()    # Grab color sensor camera's image view
    return colorSensor.imageGetGray(img, colorSensor.getWidth(), 0, 0)    # Return grayness of the only pixel (0-255)
    
def checkVic(img):
    img = np.frombuffer(img, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))  # Convert img to RGBA format (for OpenCV)

    # SIFT
    queryKP, queryDesc = detector.detectAndCompute(img, None)

    objMatched = []
    kpMatched = []
    for i, (trainKP, trainDesc) in enumerate(sift_features):
        kp_matched = 0
        matches=flann.knnMatch(queryDesc,trainDesc,k=2)
        for m, n in matches:
            if (m.distance < 0.75 * n.distance):
                kp_matched +=1
        if kp_matched >= MIN_MATCH_COUNT:
            objMatched.append(objs[i])
            kpMatched.append(kp_matched)

    if kpMatched:
        idx = 0
        max = 0
        for i, kp in enumerate(kpMatched):
            print(f'Matched key points: {objMatched[i]} {kp}')
            if kp>max:
                max = kp
                idx = i
        return objMatched[i]
    return False
    
def report(victimType):
    # Struct package to be sent to supervisor to report victim/hazard
    # First four bytes store robot's x coordinate
    # Second four bytes store robot's z coordinate
    # Last byte stores type of victim
    #     Victims: H, S, U, T
    #     Hazards: F, P, C, O
    wheel_left.setVelocity(0)   # Stop for 1 second
    wheel_right.setVelocity(0)
    delay(1300)
    victimType = bytes(victimType, "utf-8")    # Convert victimType to character for struct.pack
    posX = int(gps.getValues()[0] * 100)    # Convert from cm to m
    posZ = int(gps.getValues()[2] * 100)
    message = struct.pack("i i c", posX, posZ, victimType)
    emitter.send(message)
    robot.step(timeStep)
    
while robot.step(timestep) != -1:
    getSurrounding()
    direction = GetRobotRotation()
    # Get Robot current tile position
    current_tile_x, current_tile_y = getRobotPosition()

    print(f"---Current Tiles: ({current_tile_x}, {current_tile_y})-----")
    # print(f"---Current Position: {pos_sensor.getValues()} -----")
    is_visited_tiles(current_tile_x, current_tile_y)
    print(FindGrid(grid, current_tile_x, current_tile_y))
    printMaze(grid)
    readings = ObstacleAvoidance()
    CheckWallPosition(direction, grid, current_tile_x, current_tile_y, readings[0], readings[1], readings[2])
    # vic = checkVic(cam.getImage())
    # if vic:
    #     report(vic) # Cannot determine type of victim, so always try 'T' for now

    # wheel_left.setVelocity(speeds[0])              # Send the speed values we have choosen to the robot
    # wheel_right.setVelocity(speeds[1])