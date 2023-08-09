from controller import Robot, Motor, DistanceSensor, Camera, Emitter, GPS
import struct
import numpy as np
import cv2 as cv
import pytesseract
import os
import matplotlib.pyplot as plt

timeStep = 32            # Set the time step for the simulation
max_velocity = 6.28      # Set a maximum velocity time constant
# VICTIM_CODES = ['H','S','U','F','P','C','O']

robot = Robot()

wheel_left = robot.getDevice("left wheel motor")   # Create an object to control the left wheel
wheel_right = robot.getDevice("right wheel motor") # Create an object to control the right wheel

#[left wheel speed, right wheel speed]
speeds = [max_velocity,max_velocity]

#Create objects for all robot sensors
leftDist = robot.getDevice("leftDist")      # Get robot's left distance sensor
leftDist.enable(timeStep)     # Enable left distance sensor

frontDist = robot.getDevice("frontDist")
frontDist.enable(timeStep)

rightDist = robot.getDevice("rightDist")
rightDist.enable(timeStep)

cam = robot.getDevice("camera")
cam.enable(timeStep)

colorSensor = robot.getDevice("color")
colorSensor.enable(timeStep)

emitter = robot.getDevice("emitter")    # Emitter doesn't need enable

gps = robot.getDevice("gps")
gps.enable(timeStep)

wheel_left.setPosition(float("inf"))
wheel_right.setPosition(float("inf"))

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
    # temp = img.copy()
    #
    # img = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Grayscale image
    #
    # img, thresh = cv.threshold(img, 140, 255,
    #                            cv.THRESH_BINARY_INV)  # Inverse threshold image (0-80 -> white; 80-255 -> black)

    # Pytesseract
    # text = pytesseract.image_to_string(img)
    # print(f'Text Detected: {text}')

    # contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # plt.subplots()
    # for cnt in contours:
    #     x, y, w, h = cv.boundingRect(cnt)

    # Cropping the text block for giving input to OCR
    # cropped = temp[y:y + h, x:x + w]
    # cv.imshow(f'Contour {i}: ', cropped)
    # cv.waitKey(0)
    # cv.destroyAllWindows()

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
    
while robot.step(timeStep) != -1:
    wheel_left.setVelocity(0)  # Stop for 1 second
    wheel_right.setVelocity(0)
    # speeds[0] = max_velocity
    # speeds[1] = max_velocity
    #
    # # Check left and right sensor to avoid walls
    # # for sensor on the left, either
    # if leftDist.getValue() < 0.05:
    #     turn_right()      # We see a wall on the left, so turn right away from the wall
    #
    # if rightDist.getValue() < 0.05:		# for sensor on the right too
    #     turn_left()
    #
    # # for front sensor
    # if frontDist.getValue() < 0.05:
    #     spin()
    #
    # # if on black, turn away
    # if getColor() < 80:
    #     spin()
    #     wheel_left.setVelocity(speeds[0])
    #     wheel_right.setVelocity(speeds[1])
    #     delay(600)

    # if sees victim, report it
    vic = checkVic(cam.getImage())
    if vic:
        report(vic) # Cannot determine type of victim, so always try 'T' for now

    # wheel_left.setVelocity(speeds[0])              # Send the speed values we have choosen to the robot
    # wheel_right.setVelocity(speeds[1])