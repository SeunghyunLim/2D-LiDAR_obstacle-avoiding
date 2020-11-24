import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from time import sleep
import cv2
import math
import numpy as np
import threading

global dv, dangle
dv = 0
dangle = 0


VALID_DIST_THRES = 1.0
IMG_WIDTH = 200
IMG_HEIGHT = 200
ORIGIN_X = IMG_WIDTH/2
ORIGIN_Y = IMG_HEIGHT/2
THRESH = 0.6
safty_THRESH = 0.3

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

prev_direction = 270

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

def drawArrow(img, direction):
	if direction == "stop":
		return img
	else:
		_x = math.cos(math.pi/180*direction)*THRESH*0.8
		_y = math.sin(math.pi/180*direction)*THRESH*0.8
		img = cv2.arrowedLine(img, ((int)(ORIGIN_X), (int)(ORIGIN_Y)), ((int)(ORIGIN_X + _x*ORIGIN_X), (int)(ORIGIN_Y + _y*ORIGIN_Y)), (0, 255, 0), 2)
		return img

def move(regions, index_angle = 270):
	direction = "f"
	f = index_angle
	fr = f + 36
	fl = f - 36
	r = f + 36 + 36 + 18
	l = f - 36 - 36 - 18
	pl = 0
	pr = 0
	b = f - 180
	safty_dist = safty_THRESH

	state_description = "Unknow  state"
	d = THRESH

	if regions["front"] > d and regions["front_left"]>d and regions["front_right"]>d:
		state_description =  "case 1 - No Obstacles"
		direction = f
	elif regions["front"] < d and regions["front_left"]>d and regions["front_right"]>d:
		if regions["front"] < safty_dist:
			if regions["left"] < regions["right"]:
				state_description =  "case 10 - Obstacles in Front too closely, move right"
				direction = r
			elif regions["left"] >= regions["right"]:
				state_description =  "case 10 - obstacles in Front too closely, move left"
				direction = l
		elif regions["front"] >= safty_dist:
			if regions["left"] < regions["right"]:
				state_description =  "case 2 - Obstacles in Front, move smoothly right"
				direction = fr
			elif regions["left"] >= regions["right"]:
				state_description =  "case 2 - obstacles in Front, move smoothly left"
				direction = fl
	elif regions["front"] > d and regions["front_left"]>d and regions["front_right"]<d:
		state_description =  "case 3 - Obstacles in Front_Right, move smoothly left"
		direction = fl
	elif regions["front"] > d and regions["front_left"]<d and regions["front_right"]>d:
		state_description =  "case 4 - Obstacles in Front_Left, move smoothly right"
		direction = fr
	elif regions["front"] < d and regions["front_left"]>d and regions["front_right"]<d:
		state_description =  "case 5 - Obstacles in Front and Front_Right, move left"
		direction = l
	elif regions["front"] < d and regions["front_left"]<d and regions["front_right"]>d:
		state_description =  "case 6 - Obstacles in Front and Front_Left, move right"
		direction = r

	# When obstacles in all front 3 areas were detected
	elif regions["front"] < d and regions["front_left"]<d and regions["front_right"]<d:
		if regions["front"] < 0.3:
			if regions["left"] > regions["right"]:
				state_description =  "case 0 - Danger, move left"
				direction = l
			elif regions["left"] <= regions["right"]:
				state_description =  "case 0 - Danger, move right"
				direction = r
		elif abs(regions["front_left"]-regions["front_right"]) < 0.05:
			state_description =  "case 7 - Stuck, move backward"
			direction = b
		elif regions["front_left"] > regions["front_right"]:
			state_description =  "case 7 - Stuck, pivot left"
			direction = pl
		elif regions["front_left"] < regions["front_right"]:
			state_description =  "case 7 - Stuck, pivot right"
			direction = pr

	elif regions["front"] > d and regions["front_left"]<d and regions["front_right"]<d:
		state_description =  "case 8 - Obstacles in Front_Right and Front_Left, move backward"
		direction = b
	elif regions["front"] > d and regions["front_left"]>d and regions["front_right"]>d and regions["left"] < d and regions["right"] >d:
		state_description =  "case 9 - Left wall detected, move smoothly right"
		direction = fr
	elif regions["front"] > d and regions["front_left"]>d and regions["front_right"]>d and regions["left"] > d and regions["right"] <d:
		state_description =  "case 9 - Right wall detected, move smoothly left"
		direction = fl

    #print(state_description)
	return state_description, direction

def smooth_direction(direction, prev_direction, grad = 0.5):
	error = direction - prev_direction
	if abs(error) < 3:
		s_direction = direction
	else:
		s_direction = prev_direction + error*grad
	if s_direction >= 360:
		s_direction = 360
	elif s_direction <= 180:
		s_direction = 180

	return s_direction

def callback(data, smoothed = True):
	global imgVerbose, img, prev_direction, dangle
	distList = data.ranges
	gRegions = LaserScanProcess(distList)
	state_description, direction = move(gRegions)
	if smoothed == True:
		direction = smooth_direction(direction, prev_direction)
		prev_direction = direction

	print(direction - 270)

	dangle = direction - 270
	
	pub = rospy.Publisher('direction', Int32, queue_size=1)
	pub.publish(dangle)

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
