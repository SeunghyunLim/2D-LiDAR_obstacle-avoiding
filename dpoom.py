import rospy
from sensor_msgs.msg import LaserScan
from time import sleep
from matplotlib import pyplot as plt
import cv2
import math
import numpy as np
import RPi.GPIO as gpio
import threading
import argparse

parser = argparse.ArgumentParser(description='input mode')
parser.add_argument('--mode', required=False, default='1', help="mode 1 or 2")
args = parser.parse_args()

MODE = (int)(args.mode)

import motor

VALID_DIST_THRES = 1.0
IMG_WIDTH = 200
IMG_HEIGHT = 200
ORIGIN_X = IMG_WIDTH/2
ORIGIN_Y = IMG_HEIGHT/2

if MODE == 1:
	SPEED = 40 #40
	SPEED2 = 30 #30
	SPEED3 = 10 #10
	steer1 = 0.4 #0.5
	steer2 = 0.8 #0.9
	steer3 = 1
	THRESH = 0.6
else:
	'''
	# extreme
	SPEED = 65 #70
	SPEED2 = 65 #70
	SPEED3 = 50 #70
	steer1 = 0.4 #0.4
	steer2 = 0.25 #0.4
	steer3 = 0.5 #0.75
	THRESH = 0.6
	'''
	# regular
	SPEED = 50 #50
	SPEED2 = 40 #50
	SPEED3 = 20 #50
	steer1 = 0.4 #0.4
	steer2 = 0.45 #0.3
	steer3 = 0.9 #0.7
	THRESH = 0.6
	#'''


img = np.zeros((IMG_HEIGHT, IMG_WIDTH,3), np.uint8)
img = cv2.circle(img, ((int)(ORIGIN_X), (int)(ORIGIN_Y)), (int)(THRESH*100), (0, 0, 255), 1)

def drawLine(img, index):
	_x = math.cos(math.pi/180*index)*THRESH
	_y = math.sin(math.pi/180*index)*THRESH
	img = cv2.line(img, ((int)(ORIGIN_X), (int)(ORIGIN_Y)), ((int)(ORIGIN_X + _x*ORIGIN_X), (int)(ORIGIN_Y + _y*ORIGIN_Y)), (0, 0, 255), 2)
	return img

if MODE == 1:
	# when use normal mode
	img = drawLine(img, 90)
	img = drawLine(img, 126)
	img = drawLine(img, 162)
	img = drawLine(img, 198)
	img = drawLine(img, 234)
	img = drawLine(img, 270)
else:
	# when use far mode
	img = drawLine(img, 90)
	img = drawLine(img, 135)
	img = drawLine(img, 165)
	img = drawLine(img, 195)
	img = drawLine(img, 225)
	img = drawLine(img, 270)

def LaserScanProcess(distList):
    temp = np.asarray(distList)
    distList = np.where(temp==0, 10, temp)
    gRegions = {
       "right" : round(min(min(distList[90:125]),10),3),
        "front_right" : round(min(min(distList[126:161]),10),3),
        "front" : round(min(min(distList[162:197]),10),3),
        "front_left" : round(min(min(distList[198:233]),10),3),
        "left" : round(min(min(distList[234:269]),10),3),
    }
    #print(gRegions)
    return gRegions

def LaserScanProcess_far(distList):
    temp = np.asarray(distList)
    distList = np.where(temp==0, 10, temp)
    gRegions = {
       "right" : round(min(min(distList[90:134]),10),3),
        "front_right" : round(min(min(distList[135:164]),10),3),
        "front" : round(min(min(distList[165:194]),10),3),
        "front_left" : round(min(min(distList[195:224]),10),3),
        "left" : round(min(min(distList[225:269]),10),3),
    }
    #print(gRegions)
    return gRegions

def turn_right(state):
    if state==1:
        motor.mvRotate(SPEED2, steer1)
    elif state==2:
        motor.mvRotate(SPEED2, steer2)
    elif state == 3:
        motor.mvRotate(SPEED3, steer3)
def turn_left(state):
    if state==1:
        motor.mvRotate(SPEED2, -steer1)
    elif state==2:
        motor.mvRotate(SPEED2, -steer2)
    elif state == 3:
        motor.mvRotate(SPEED3, -steer3)
def move_forward():
    motor.mvStraight(SPEED)
def stop():
    motor.mvStraight(0)

def change_state(state):
    if state == "right":
        turn_right(2)
    elif state == "left":
        turn_left(2)
    elif state == "forward":
        move_forward()
    elif state == "forward_left":
    	turn_left(1)
    elif state == "forward_right":
    	turn_right(1)
    elif state == "pivot_left":
    	turn_left(3)
    elif state == "pivot_right":
    	turn_right(3)
    else:
        stop()


def move_far(regions):
	farthest = max(regions.keys(), key=(lambda k: regions[k]))
	if farthest == "front":
		state_description =  "front"
		change_state("forward")
	elif farthest == "front_right":
		state_description =  "front_right"
		change_state("right")
	elif farthest == "front_left":
		state_description =  "front_left"
		change_state("left")
	elif farthest == "right":
		state_description =  "right"
		change_state("pivot_right")
	elif farthest == "left":
		state_description =  "left"
		change_state("pivot_left")
	#print(state_description)

def move(regions):
    d = THRESH # m
    if regions["front"] > d and regions["front_left"]>d and regions["front_right"]>d:
        state_description =  "case 1 - Nothing"
        change_state("forward")
    elif regions["front"] < d and regions["front_left"]>d and regions["front_right"]>d:
        if regions["left"] < regions["right"]:
        	state_description =  "case 2 - Front, turn right"
        	change_state("pivot_right")
        elif regions["left"] >= regions["right"]:
        	state_description =  "case 2 - Front, turn left"
        	change_state("pivot_left")
        #state_description =  "case 2 - Front"
        #change_state("stop")
    elif regions["front"] > d and regions["front_left"]>d and regions["front_right"]<d:
        state_description =  "case 3 - Front_Right"
        change_state("forward_left")
    elif regions["front"] > d and regions["front_left"]<d and regions["front_right"]>d:
        state_description =  "case 4 - Front_Left"
        change_state("forward_right")
    elif regions["front"] < d and regions["front_left"]>d and regions["front_right"]<d:
        state_description =  "case 5 - Front and Right"
        change_state("left")
    elif regions["front"] < d and regions["front_left"]<d and regions["front_right"]>d:
        state_description =  "case 6 - Front and Left"
        change_state("right")
    elif regions["front"] < d and regions["front_left"]<d and regions["front_right"]<d:
    	if regions["front"] < 0.25:
    		if regions["left"] > regions["right"]:
    			state_description =  "case 0 - Danger, turn left"
    			change_state("pivot_left")
    		elif regions["left"] <= regions["right"]:
    			state_description =  "case 0 - Danger, turn right"
    			change_state("pivot_right")
        elif abs(regions["front_left"]-regions["front_right"]) < 0.05:
        	state_description =  "case 7 - All, Forward"
        	change_state("forward")
        elif regions["front_left"] > regions["front_right"]:
        	state_description =  "case 7 - All, pivot left"
        	change_state("pivot_left")
        elif regions["front_left"] < regions["front_right"]:
        	state_description =  "case 7 - All, pivot right"
        	change_state("pivot_right")
        else:
        	state_description =  "case 7 - All, else"
        #change_state("stop")
    elif regions["front"] > d and regions["front_left"]<d and regions["front_right"]<d:
        state_description =  "case 8 - Front_Right and Front_Left"
        change_state("forward")
    elif regions["front"] > d and regions["front_left"]>d and regions["front_right"]>d and regions["left"] < d and regions["right"] >d:
        state_description =  "case 9 - Left wall detected"
        change_state("forward_right")
    elif regions["front"] > d and regions["front_left"]>d and regions["front_right"]>d and regions["left"] > d and regions["right"] <d:
        state_description =  "case 9 - Right wall detected"
        change_state("forward_left")
    else:
        state_description = "Unknow  state"
        change_state("stop")
    print(state_description)

def callback(data):
    global MODE
    global imgVerbose, img
    distList = data.ranges
    if MODE == 1:
    	gRegions = LaserScanProcess(distList)
    	move(gRegions)
    else:
    	gRegions = LaserScanProcess_far(distList)
    	move_far(gRegions)
    #print('---')
    if imgVerbose:
    	img2= img.copy()
        for idx, elem in enumerate(distList):
            if elem > VALID_DIST_THRES :
                continue
            _x = math.cos(math.pi/180*idx)*elem
            _y = math.sin(math.pi/180*idx)*elem
            img2 = cv2.circle(img2, ((int)(ORIGIN_X + _x*ORIGIN_X), (int)(ORIGIN_Y + _y*ORIGIN_Y)), 2, (0, 255, 0), -1)
        img2 = cv2.circle(img2, ((int)(ORIGIN_X), (int)(ORIGIN_Y)), 5, (0,0,255), -1)
        img2 = cv2.resize(img2, None, fx=4, fy=4)
        cv2.imshow("pointcloud", img2)
        cv2.waitKey(1)


def listener():
	rospy.init_node('listner', anonymous=True)
	rospy.Subscriber("/scan", LaserScan, callback)
	rospy.spin()

if __name__ == '__main__':
    try:
        imgVerbose = False
        #drivingThread = threading.Thread(target = driving)
        #drivingThread.start()
        print("FUCK")
        listener()

    # press 'k', then ctrl-C to shutdown
    except KeyboardInterrupt:
        print "Ctrl-C - quit"
    # Cleanup GPIO
    finally:
        gpio.cleanup()
        print("GPIO CLEANUP COMPLETE")
