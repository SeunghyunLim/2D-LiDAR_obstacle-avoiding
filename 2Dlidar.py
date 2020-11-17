import rospy
from sensor_msgs.msg import LaserScan
from time import sleep
from matplotlib import pyplot as plt
import cv2
import math
import numpy as np
import threading

VALID_DIST_THRES = 1.0
IMG_WIDTH = 200
IMG_HEIGHT = 200
ORIGIN_X = IMG_WIDTH/2
ORIGIN_Y = IMG_HEIGHT/2
THRESH = 0.6

img = np.zeros((IMG_HEIGHT, IMG_WIDTH,3), np.uint8)
img = cv2.circle(img, ((int)(ORIGIN_X), (int)(ORIGIN_Y)), (int)(THRESH*100), (0, 0, 255), 1)

def drawLine(img, index):
	_x = math.cos(math.pi/180*index)*THRESH
	_y = math.sin(math.pi/180*index)*THRESH
	img = cv2.line(img, ((int)(ORIGIN_X), (int)(ORIGIN_Y)), ((int)(ORIGIN_X + _x*ORIGIN_X), (int)(ORIGIN_Y + _y*ORIGIN_Y)), (0, 0, 255), 2)
	return img

img = drawLine(img, 180)
img = drawLine(img, 216)
img = drawLine(img, 252)
img = drawLine(img, 288)
img = drawLine(img, 324)
img = drawLine(img, 360)

def LaserScanProcess(distList):
    temp = np.asarray(distList)
    distList = np.where(temp==0, 10, temp)
    gRegions = {
        "left" : round(min(min(distList[234:270]),10),3),
        "front_left" : round(min(min(distList[198:233]),10),3),
		"front" : round(min(min(distList[162:197]),10),3),
        "front_right" : round(min(min(distList[126:161]),10),3),
        "right" : round(min(min(distList[90:125]),10),3)
    }
    return gRegions

def drawArrow(img, direction, index = 270):
	if direction == "f":
		index
	elif direction == "fr":
		index += 36
	elif direction == "fl":
		index -= 36
	elif direction == "r":
		index += 36 + 36
	elif direction == "l":
		index -= 36 + 36

	_x = math.cos(math.pi/180*index)*THRESH*0.8
	_y = math.sin(math.pi/180*index)*THRESH*0.8

	img = cv2.arrowedLine(img, ((int)(ORIGIN_X), (int)(ORIGIN_Y)), ((int)(ORIGIN_X + _x*ORIGIN_X), (int)(ORIGIN_Y + _y*ORIGIN_Y)), (0, 255, 0), 2)
	return img

def move(regions):
	direction = "f"
	state_description = "Unknow  state"
	d = THRESH

	if regions["front"] > d and regions["front_left"]>d and regions["front_right"]>d:
		state_description =  "case 1 - No Obstacles"
		direction = "f"
	elif regions["front"] < d and regions["front_left"]>d and regions["front_right"]>d:
		if regions["left"] < regions["right"]:
			state_description =  "case 2 - Obstacles in Front, move right"
			direction = "r"
		elif regions["left"] >= regions["right"]:
			state_description =  "case 2 - obstacles in Front, move left"
			direction = "l"
	elif regions["front"] > d and regions["front_left"]>d and regions["front_right"]<d:
		state_description =  "case 3 - Obstacles in Front_Right, move smoothly left"
		direction = "fl"
	elif regions["front"] > d and regions["front_left"]<d and regions["front_right"]>d:
		state_description =  "case 4 - Obstacles in Front_Left, move smoothly right"
		direction = "fr"
	elif regions["front"] < d and regions["front_left"]>d and regions["front_right"]<d:
		state_description =  "case 5 - Obstacles in Front and Front_Right, move left"
		direction = "l"
	elif regions["front"] < d and regions["front_left"]<d and regions["front_right"]>d:
		state_description =  "case 6 - Obstacles in Front and Front_Left, move right"
		direction = "r"

	# When obstacles in all front 3 areas were detected
	elif regions["front"] < d and regions["front_left"]<d and regions["front_right"]<d:
		if regions["front"] < 0.3:
			if regions["left"] > regions["right"]:
				state_description =  "case 0 - Danger, move left"
				direction = "l"
			elif regions["left"] <= regions["right"]:
				state_description =  "case 0 - Danger, move right"
				direction = "r"
		elif abs(regions["front_left"]-regions["front_right"]) < 0.05:
			state_description =  "case 7 - Stuck, so move left"
			direction = "l"
		elif regions["front_left"] > regions["front_right"]:
			state_description =  "case 7 - All, pivot left"
			direction = "pl"
		elif regions["front_left"] < regions["front_right"]:
			state_description =  "case 7 - All, pivot right"
			direction = "pr"
        #else:
        #	state_description =  "case 7 - All, else"

	elif regions["front"] > d and regions["front_left"]<d and regions["front_right"]<d:
		state_description =  "case 8 - Obstacles in Front_Right and Front_Left, Stop"
		direction = "stop"
	elif regions["front"] > d and regions["front_left"]>d and regions["front_right"]>d and regions["left"] < d and regions["right"] >d:
		state_description =  "case 9 - Left wall detected, move smoothly right"
		direction = "fr"
	elif regions["front"] > d and regions["front_left"]>d and regions["front_right"]>d and regions["left"] > d and regions["right"] <d:
		state_description =  "case 9 - Right wall detected, move smoothly left"
		direction = "fl"

    #print(state_description)
	return state_description, direction


def callback(data):
	global myRobot
	global imgVerbose, img
	distList = data.ranges
	gRegions = LaserScanProcess(distList)
	state_description, direction = move(gRegions)
	print(state_description)

	if imgVerbose:
		img2 = img.copy()
		for idx, elem in enumerate(distList):
			if elem > VALID_DIST_THRES :
				continue
			_y = math.cos(math.pi/180*idx)*elem
			_x = math.sin(math.pi/180*idx)*elem
			img2 = cv2.circle(img2, ((int)(ORIGIN_X + _x*ORIGIN_X), (int)(ORIGIN_Y + _y*ORIGIN_Y)), 1, (0, 255, 0), -1)
		img2 = cv2.circle(img2, ((int)(ORIGIN_X), (int)(ORIGIN_Y)), 5, (0,0,255), -1)
		img2 = drawArrow(img2, direction)
		img2 = cv2.resize(img2, None, fx=3, fy=3)
		cv2.imshow("pointcloud", img2)
		cv2.waitKey(1)


def listener():
	rospy.init_node('listner', anonymous=True)
	rospy.Subscriber("/scan", LaserScan, callback)
	rospy.spin()



if __name__ == "__main__":
	try:
		imgVerbose = True
		listener()

	except KeyboardInterrupt:
		print("Interrupted by keyboard")
