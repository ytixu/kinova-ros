#! /usr/bin/env python

import rospy
import numpy as np
import sys
import copy
import cv2
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from move_robot import moveJoint, moveFingers

prefix = 'j2n6s300'
nbJoints = 6
nbfingers = 3
THR = 0.01
NORM = np.pi*2
joint_states = None
arm_image = None

cv_bridge = CvBridge()

def update_image(image):
	# print image
	global arm_image
	try:
		arm_image = cv_bridge.imgmsg_to_cv2(image, "bgr8")
		cv2.imshow('arm-view', arm_image)
		cv2.waitKey(1)
	except CvBridgeError as e:
		print(e)

def update_joint_state(joint_states_msg):
	global joint_states
	joint_states = list(joint_states_msg.position)

def compare_diff(grasp, joints):
	diff = np.linalg.norm(np.array(joints) - np.array(joint_states[:6])%NORM)
	print diff
	return diff > THR

# e.g.: n8j3j1jn21j8j14o
def convert(line):
	line = line.strip('\n')
	graps = [0,0,0] if line[-1] == 'o' else [1,1,1]
	line = line[:-1].replace('n', '-').split('j')
	joints = [float(a)%NORM for a in line]

	print joints
	print graps
	attemps = 0
	while (compare_diff(graps, joints) and attemps < 5):
		moveJoint(joints, prefix, nbJoints)
		moveFingers(graps, prefix, nbfingers)
		attemps += 1

 # cat <file> | rosrun ...

if __name__ == '__main__':
	rospy.init_node('pose_picture_taker', anonymous=True)
	rospy.Subscriber('/j2n6s300/joint_states', JointState, update_joint_state)
	rospy.Subscriber('/xtion/rgb/image_raw', Image, update_image)
	rospy.sleep(1)
	for i, line in enumerate(sys.stdin):
		# sys.stdout.write(line)
		convert(line)
		image = copy.copy(arm_image)
		cv2.imshow('arm', image)
		cv2.waitKey(1)
		cv2.imwrite('./data/arm_%d.jpg' % (i), image)
