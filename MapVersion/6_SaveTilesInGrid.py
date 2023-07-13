import time
from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math
import random
import sys

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
    if lds.getValue() < 0.3 :
        #print(f'Left Wall Detected: {lds.getValue()}')
        print(f'Left Wall Detected')
         
    if rds.getValue() < 0.3 :
        #print(f'Right Wall Detected: {rds.getValue()}') 
        print(f'Right Wall Detected') 
        
    if fds.getValue() < 0.3 :
        #print(f'Front Wall Detected: {fds.getValue()}') 
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

#New empty list
visited_cells = []
already_visited = []

# Original Grid [Visited, North, East, South, West]
grid = [[0, 1, 0, 0, 1], [0, 1, 0, 0, 0], [0, 1, 0, 0, 0],[0, 1, 0, 0, 0],[0, 1, 0, 0, 0],[0, 1, 0, 0, 0],[0, 1, 0, 0, 0],[0, 1, 1, 0, 0], #starting grid
        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 0, 0],
        [0, 0, 0, 1, 1], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 0, 0, 0],[0, 0, 1, 1, 0]]

def printMaze(maze):
    print("__________________________________________________________________________________")
    for i in range(8):
        x = i*8
        if (maze[x][0] == 0):
            v1 = "?"
        else:
            v1 = "V"
        if (maze[x+1][0] == 0):
            v2 = "?"
        else:
            v2 = "V"
        if (maze[x+2][0] == 0):
            v3 = "?"
        else:
            v3 = "V"
        if (maze[x+3][0] == 0):
            v4 = "?"
        else:
            v4 = "V"
        if (maze[x+4][0] == 0):
            v5 = "?"
        else:
            v5 = "V"
        if (maze[x+5][0] == 0):
            v6 = "?"
        else:
            v6 = "V"
        if (maze[x+6][0] == 0):
            v7 = "?"
        else:
            v7 = "V"
        if (maze[x+7][0] == 0):
            v8 = "?"
        else:
            v8 = "V"
        print("|  "+ str(maze[x][1]) +"\t  " +str(maze[x+1][1])+"\t  " +str(maze[x+2][1]) +"\t  " +str(maze[x+3][1])+ "\t  " +str(maze[x+4][1])+ "\t  " +str(maze[x+5][1])+ "\t  " +str(maze[x+6][1])+ "\t  " +str(maze[x+7][1])+ "    |")
        print("|" +str(maze[x][4]) + " " +v1+" " + str(maze[x][2])+"\t" +str(maze[x+1][4])+ " " +v2+" " + str(maze[x+1][2])
                +"\t" +str(maze[x+2][4])+ " " +v3+" " + str(maze[x+2][2])
                +"\t" +str(maze[x+3][4]) + " " +v4+" " + str(maze[x+3][2])
                +"\t" +str(maze[x+4][4]) + " " +v5+" " + str(maze[x+4][2])
                +"\t" +str(maze[x+5][4]) + " " +v6+" " + str(maze[x+5][2])
                +"\t" +str(maze[x+6][4]) + " " +v7+" " + str(maze[x+6][2])
                +"\t" +str(maze[x+7][4]) + " " +v8+" " + str(maze[x+7][2]) +"  |")
        print("|  "+ str(maze[x][3]) +"\t  " +str(maze[x+1][3])+"\t  " +str(maze[x+2][3]) +"\t  " +str(maze[x+3][3])+"\t  " +str(maze[x+4][3])+"\t  " +str(maze[x+5][3])+"\t  " +str(maze[x+6][3])+"\t  " +str(maze[x+7][3])+ "    |")
        if(i==7):
            print("|__________________________________________________________________________________|\n")
        else:
            print("|                                                                                  |")

def getRobotPosition():
    position = pos_sensor.getValues()
    # Calculate the tile position based on the coordinates
    x_pos = round(position[0],1)
    y_pos = round(position[2],1)
    
    # Dividing according to the tile size of map (12cm)
    start_tile_x = round((x_pos / tile_size)) 
    start_tile_y = round((y_pos / tile_size)) 

    #x = -3 ; y = -4
    diff_x = 3
    diff_y = 4

    # Move the robot in the calculated direction
    new_x = start_tile_x + diff_x
    new_y = start_tile_y + diff_y

    print('Updated X Position:' + str(new_x))
    print('Updated Y Position:' + str(new_y))

    return new_x, new_y

def getPositionSensors():#function for getting getting position sensors
    return lds.getValue(), rds.getValue()

#Index value for grid
def setItem(a,c,r,value): 
    a[r*MAZE_COLUMNS+c] = value 
    print('This Value', value)

#Get corresponding of current grid
def FindGrid(a,c,r): 
    return a[r*MAZE_COLUMNS+c] 

# Check if tile is visited
def is_visited_tiles(current_tile_x, current_tile_y):
    grid_position = FindGrid(grid,current_tile_x,current_tile_y)
    if not grid_position[0] == 1:
        visited_cells.append((current_tile_x, current_tile_y))
        grid_position[0] = 1
        print('Tile Saved into List')
    else:
        print('Tile Visited.')
    
def UpdateMaze(current_tile_x,current_tile_y, L, F, R, direction):
    grid_list = FindGrid(grid,current_tile_x,current_tile_y)
    grid_list[0] = 1 #mark cell visited
    n = (current_tile_x, current_tile_y)

    if n not in already_visited:
        # mark internal walls based on sensor readings
        if direction == 'N':
            grid_list[4] = L       # LEFT sensor = WEST wall
            grid_list[1] = F       # FRONT sensor = NORTH wall
            grid_list[2] = R       # RIGHT sensor = WEST wall
        elif direction == 'E':
            grid_list[1] = L       # LEFT sensor = NORTH wall
            grid_list[2] = F       # FRONT sensor = EAST wall
            grid_list[3] = R       # RIGHT sensor = SOUTH wall
        elif direction == 'S':
            grid_list[2] = L       # LEFT sensor = EAST wall
            grid_list[3] = F       # FRONT sensor = SOUTH wall
            grid_list[4] = R       # RIGHT sensor = WEST wall
        elif direction == 'W':
            grid_list[3] = L       # LEFT sensor = SOUTH wall
            grid_list[4] = F       # FRONT sensor = WEST wall
            grid_list[1] = R       # RIGHT sensor = NORTH wall
        already_visited.append(n)

    
def ObstacleAvoidance():
    front = fds.getValue()
    left = lds.getValue()
    right = rds.getValue()
    L = 1 if left < 0.3 else 0
    F = 1 if front < 0.3 else 0
    R = 1 if right < 0.3 else 0
    #print('L(', L,') F(', F,') R(', R,')')
    return (L,F,R)    
    

while robot.step(timestep) != -1:
    getSurrounding()
    GetRobotRotation()
    #Get Robot current tile position
    current_tile_x, current_tile_y = getRobotPosition()

    print(f"---Current Tiles: ({current_tile_x}, {current_tile_y})-----")
    #print(f"---Current Position: {pos_sensor.getValues()} -----")
    is_visited_tiles(current_tile_x, current_tile_y)
    print(FindGrid(grid,current_tile_x,current_tile_y))
    printMaze(grid)
    #readings = ObstacleAvoidance()
    #UpdateMaze(current_tile_x,current_tile_y, readings[0],readings[1],readings[2],GetRobotRotation())
    #CheckWalls(readings[0],readings[1],readings[2])



