#!/usr/bin/env python
# Convert ARMarker and KinovaPose into geometry_msgs/Transform

import rospy
import copy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseStamped
from visp_hand2eye_calibration.msg import TransformArray
from ar_pose.msg import ARMarker

from kinova_demo.robot_control_modules import joint_position_client
import math

# move robot
prefix = 'j2n6s300_'
nbJoints = 6

positions = [
	[-46.432098388671875, 215.6886749267578, 86.08859252929689, -149.4498291015625, 81.1849365234375, 103.15209960937506, 0.0],
	[15.679229736328125, 188.75166320800778, 35.235862731933594, -140.19998168945312, 122.50994873046875, 96.69244384765625, 0.0],
	[12.860763549804688, 278.3850402832031, 53.151878356933594, -131.52340698242188, 127.61816406250006, 166.7235107421875, 0.0],
	[5.133270263671875, 261.79852294921875, 72.9647674560547, -169.64387512207028, 134.02059936523438, 123.76104736328125, 0.0],
	[-3.260101318359375, 283.1480407714844, 52.1471939086914, -118.79060363769531, 116.89929199218756, 177.69879150390625, 0.0],
	[-48.65452575683594, 261.61737060546875, 73.48995971679688, -169.78704833984375, 133.66873168945312, 123.27679443359375, 0.0],
	[-26.285720825195312, 268.08740234375, 127.7167205810547, -157.8896026611328, 87.6744384765625, 111.34942626953125, 0.0],
	[-27.396713256835938, 232.6794891357422, 120.48322296142578, -139.56517028808594, 64.89132690429688, 94.58038330078125, 0.0],
	[-69.70278167724611, 207.78593444824216, 67.14425659179688, -151.40565490722656, 94.9163208007813, 103.91973876953125, 0.0],
	[-3.896133422851591, 175.52920532226565, 77.6861572265625, -127.91886901855469, 78.33395385742188, 67.5056762695312, 0.0]
]


def move_robot():
	for pos in positions:
		# pos = [p % 360 if p > 0 else p for p in pos]
		# print pos
		raw_input('Next movement.')
		result = joint_position_client(pos, prefix)


class CalibrateConverter:

	def __init__(self):
		self.pub_cam = rospy.Publisher('world_effector', TransformArray, queue_size=10)
		self.pub_arm = rospy.Publisher('camera_object', TransformArray, queue_size=10)

		self.N = 11
		self.arm_poses = [None]*self.N
		self.cam_poses = [None]*self.N
		self.arm_poses_l = 0
		self.cam_poses_l = 0

		rospy.Subscriber('ar_pose_marker', ARMarker, self.publish_ar_marker_pose)
		rospy.Subscriber('j2n6s300_driver/out/tool_pose', PoseStamped, self.publish_kinova_arm_pose)

	def publish_ar_marker_pose(self, data):
		newData = Transform()
		newData.translation.x = data.pose.pose.position.x
		newData.translation.y = data.pose.pose.position.y
		newData.translation.z = data.pose.pose.position.z
		newData.rotation.x = data.pose.pose.orientation.x
		newData.rotation.y = data.pose.pose.orientation.y
		newData.rotation.z = data.pose.pose.orientation.z
		newData.rotation.w = data.pose.pose.orientation.w

		self.cam_poses[self.cam_poses_l] = newData
		self.cam_poses_l += 1

		if self.cam_poses_l == self.N:
			self.cam_poses_l = 0
			pub_data = TransformArray()
			pub_data.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
			pub_data.transforms = copy.copy(self.cam_poses)
			self.pub_cam.publish(pub_data)
			self.cam_poses = [None]*self.N


	def publish_kinova_arm_pose(self, data):
		newData = Transform()
		newData.translation.x = data.pose.position.x
		newData.translation.y = data.pose.position.y
		newData.translation.z = data.pose.position.z
		newData.rotation.x = data.pose.orientation.x
		newData.rotation.y = data.pose.orientation.y
		newData.rotation.z = data.pose.orientation.z
		newData.rotation.w = data.pose.orientation.w

		self.arm_poses[self.arm_poses_l] = newData
		self.arm_poses_l += 1

		if self.arm_poses_l == self.N:
			self.arm_poses_l = 0
			pub_data = TransformArray()
			pub_data.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
			pub_data.transforms = copy.copy(self.arm_poses)
			self.pub_arm.publish(pub_data)
			self.arm_poses = [None]*self.N


if __name__ == '__main__':
	try:
		rospy.init_node('calibrator', anonymous=True)
		# CalibrateConverter()
		move_robot()
		# rospy.spin()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"