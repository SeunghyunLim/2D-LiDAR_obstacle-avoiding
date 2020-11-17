import rospy
from sensor_msgs.msg import LaserScan
from time import sleep
from matplotlib import pyplot as plt
import cv2
import math
import numpy as np


VALID_DIST_THRES = 1.0
IMG_WIDTH = 200
IMG_HEIGHT = 200
ORIGIN_X = IMG_WIDTH/2
ORIGIN_Y = IMG_HEIGHT/2

def callback(data):
	global myRobot
	global imgVerbose
	if imgVerbose:
		img = np.zeros((IMG_HEIGHT, IMG_WIDTH,3), np.uint8)
		for idx, elem in enumerate(data.ranges):
			if elem > VALID_DIST_THRES :
				continue
			_y = math.cos(math.pi/180*idx)*elem
			_x = math.sin(math.pi/180*idx)*elem
			img = cv2.circle(img, ((int)(ORIGIN_X + _x*ORIGIN_X), (int)(ORIGIN_Y + _y*ORIGIN_Y)), 1, (0, 255, 0), -1)
		img = cv2.circle(img, ((int)(ORIGIN_X), (int)(ORIGIN_Y)), 5, (0,0,255), -1)
		cv2.imshow("pointcloud", img)
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
