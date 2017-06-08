#!/usr/bin/env python
# Convert ARMarker and KinovaPose into geometry_msgs/Transform

import rospy
import copy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseStamped
from visp_hand2eye_calibration.msg import TransformArray
from ar_pose.msg import ARMarker

from kinova_demo.robot_control_modules import joint_position_client
from kinova_demo.joints_action_client import unitParser

# move robot
prefix = 'j2n6s300_'
nbJoints = 6

positions = [
	[-0.9, 3.8, 1.5, -2.6, 7.7, 8.1],
	[0.3, 3.3, 0.6, -2.4, 8.4, 8],
	[0.3, 4.9, 1, -2.3, 8.5, 9.2],
	[-0.1, 4.6, 1.3, -3, 8.6, 8.4],
	[-0.1, 5, 0.9, -2, 8.3, 9.4],
	[-0.8, 4.5, 1.3, -3, 8.6, 8.4],
	[-0.4, 4.5, 2.2, -3, 8.6, 8.4],
	[-0.4, 4.7, 2.2, -2.8, 7.8, 8.2],
	[-0.4, 4.1, 2.1, -2.4, 7.4, 8],
	[-1.2, 3.6, 1.2, -2.6, 7.9, 8],
	[-0.1, 3, 1.3, -2.2, 7.6, 7.5]
]

def move_robot():
	for pos in positions:
		# pos = [p % 360 if p > 0 else p for p in pos]
		pos_in_deg, _ = unitParser('radian', pos, False)
		pos_in_deg.append(0)
		# print pos_in_deg
		# raw_input('Next movement.')
		result = joint_position_client(pos_in_deg, prefix)
		print result


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