#!/usr/bin/env python
# Convert ARMarker and KinovaPose into geometry_msgs/Transform

import rospy
import copy
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from ar_pose.msg import ARMarker
from kinova_msgs.msg import KinovaPose

class CalibrateConverter:

	def __init__(self):
		self.pub_arm = rospy.Publisher('world_effector', TransformArray, queue_size=10)
		# self.pub_cam = rospy.Publisher('calibrator/camera_view', TransformArray, queue_size=10)

		self.N = 11
		self.arm_poses = [None]*self.N
		self.cam_poses = [None]*self.N
		self.arm_poses_l = 0
		self.cam_poses_l = 0

		rospy.Subscriber('ar_pose_marker', ARMarker, self.publish_ar_marker_pose)
		# rospy.Subscriber('j2n6s300_driver/out/tool_wrench', KinovaPose, self.publish_kinova_arm_pose)

	def publish_ar_marker_pose(self, data):
		newData = Transform()
		# newData.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
		newData.translation.x = data.pose.pose.position.x
		newData.translation.y = data.pose.pose.position.y
		newData.translation.z = data.pose.pose.position.z
		newData.rotation.x = data.pose.pose.orientation.x
		newData.rotation.y = data.pose.pose.orientation.y
		newData.rotation.z = data.pose.pose.orientation.z

		self.arm_poses[self.arm_poses_l] = newData
		self.arm_poses_l += 1

		if self.arm_poses_l == self.N:
			self.arm_poses_l = 0
			pub_data = TransformArray()
			pub_data.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
			pub_data.transforms = copy.copy(self.arm_poses)
			self.pub_arm.publish(pub_data)
			self.arm_poses = [None]*self.N



	# def publish_kinova_arm_pose(self, data):

if __name__ == '__main__':
	try:
		rospy.init_node('calibrator', anonymous=True)
		CalibrateConverter()
		rospy.spin()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"