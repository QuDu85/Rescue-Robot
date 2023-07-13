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
    temp = 0 
    if lds.getValue() < 0.3 :
        #print(f'Left Wall Detected: {lds.getValue()}')
        print(f'Left Wall Detected')
        temp = 1 
    if rds.getValue() < 0.3 :
        #print(f'Right Wall Detected: {rds.getValue()}') 
        print(f'Right Wall Detected') 
        temp = 1
    if fds.getValue() < 0.3 :
        #print(f'Front Wall Detected: {fds.getValue()}') 
        print(f'Front Wall Detected')
        temp = 1 

    return temp

# detect the angle where robot is facing
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

# Map details
tile_size = 0.12
maze_width = 8 #column
maze_length = 8 #row

maze = [[False] * maze_width for _ in range(maze_length)]

# Empty list of visited tiles
visited_tiles_list = []

# Get the robot's current tile position in the maze
def get_robot_tile_position():
    # Get the position values from the sensor
    position = pos_sensor.getValues()

    # Calculate the tile position based on the coordinates
    x_pos = round(position[0],1)
    y_pos = round(position[2],1)
    #print(x_pos)
    #print(y_pos)
    tile_x = round((x_pos / tile_size)) #column
    tile_y = round((y_pos / tile_size)) #row
    #print(tile_x)
    #print(tile_y)
    return tile_x, tile_y

# Mark a tile as visited in the maze
def mark_tile_as_visited(tile_x, tile_y):
    maze[tile_y][tile_x] = True

# Check if a tile has been visited in the maze
def is_tile_visited(tile_x, tile_y):
    return maze[tile_y][tile_x]


while robot.step(timestep) != -1:
    getSurrounding()
    GetRobotRotation()
    #Get Robot current tile position
    current_tile_x, current_tile_y = get_robot_tile_position()
    print(f"-----------Current Tiles: ({current_tile_x}, {current_tile_y}) --------------------")
    print(f"---Current Position: {pos_sensor.getValues()} -----")
    
    #Count unique number of tiles
    to_set = set(visited_tiles_list)
    num_tiles = len(to_set)
    print(num_tiles)
    print(visited_tiles_list)
    
    # Check if the current tile has been visited
    if not is_tile_visited(current_tile_x, current_tile_y):
        # Mark the current tile as visited
        mark_tile_as_visited(current_tile_x, current_tile_y)
        visited_tiles_list.append((current_tile_x, current_tile_y))
        print(f"-----------Visited tile: ({current_tile_x}, {current_tile_y})===========")
        

