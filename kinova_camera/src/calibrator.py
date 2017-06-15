#!/usr/bin/env python
# Convert ARMarker and KinovaPose into geometry_msgs/Transform

import rospy
import rospkg
import message_filters
import numpy as np
from scipy.spatial.distance import euclidean as scipy_distance
import tf.transformations as tf
import tf_conversions.posemath as tf_conv

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


def get_cfg():
	import yaml
	file_name = rospkg.RosPack().get_path('kinova_camera')+'/config/calibration.yaml'
	cWe = None
	with open(file_name, 'r') as ymlfile:
		cfg = yaml.load(ymlfile)

		trans = [cfg['translation']['x'],
				cfg['translation']['y'],
				cfg['translation']['z']]
		rot = [cfg['rotation']['x'],
				cfg['rotation']['y'],
				cfg['rotation']['z'],
				cfg['rotation']['w']]

		cWe = tf_conv.toMatrix(tf_conv.fromTf([trans, rot]))
		print cWe

	return cWe


class CalibrateConverter:

	def __init__(self):
		self.pub_arm = rospy.Publisher('world_effector', Transform, queue_size=1)
		self.pub_cam = rospy.Publisher('camera_object', Transform, queue_size=1)

		self.N = 200000
		self.count = 0
		self.prev_marker = None
		self.prev_pose = None
		self.dist_th = 0.01

		self.cWe = None

		marker_sub = message_filters.Subscriber('ar_pose_marker', ARMarker)
		pose_sub = message_filters.Subscriber('j2n6s300_driver/out/tool_pose', PoseStamped)

		ts = message_filters.ApproximateTimeSynchronizer([marker_sub, pose_sub], 10, 0.1)
		ts.registerCallback(self.callback)

	def callback(self, marker, pose):
		print self.count
		if self.count >= self.N:
			return
		else:
			marker_data = self.get_marker_data(marker)

			if marker_data == None:
				return

			pose_data = self.get_pose_data(pose)

			self.count += 1
			self.pub_cam.publish(marker_data)
			self.pub_arm.publish(pose_data)


	def get_marker_data(self, data):
		new_marker = [data.pose.pose.position.x,
			data.pose.pose.position.y,
			data.pose.pose.position.z,
			data.pose.pose.orientation.x,
			data.pose.pose.orientation.y,
			data.pose.pose.orientation.z,
			data.pose.pose.orientation.w]

		if self.prev_marker != None:
			dist = scipy_distance(self.prev_marker, new_marker)
			# print 'marker ', dist
			if dist < self.dist_th:
				return None

		self.prev_marker = new_marker

		marker_mat = tf_conv.toMatrix(tf_conv.fromMsg(data.pose.pose))
		marker_mat = np.linalg.pinv(marker_mat)
		data = tf_conv.toMsg(tf_conv.fromMatrix(marker_mat))

		newData = Transform()
		newData.translation.x = data.position.x
		newData.translation.y = data.position.y
		newData.translation.z = data.position.z
		newData.rotation.x = data.orientation.x
		newData.rotation.y = data.orientation.y
		newData.rotation.z = data.orientation.z
		newData.rotation.w = data.orientation.w

		return newData


	def get_pose_data(self, data):
		newData = Transform()
		newData.translation.x = data.pose.position.x
		newData.translation.y = data.pose.position.y
		newData.translation.z = data.pose.position.z
		newData.rotation.x = data.pose.orientation.x
		newData.rotation.y = data.pose.orientation.y
		newData.rotation.z = data.pose.orientation.z
		newData.rotation.w = data.pose.orientation.w

		return newData


	def test(self, marker, arm_pose):

		if self.cWe == None:
			self.cWe = get_cfg()

		marker_mat = tf_conv.toMatrix(tf_conv.fromMsg(marker.pose.pose))
		marker_mat = np.linalg.pinv(marker_mat)
		arm_mat = tf_conv.toMatrix(tf_conv.fromMsg(arm_pose.pose))

		camera_trans = np.dot(np.dot(marker_mat, self.cWe), arm_mat)

		print tf_conv.toMsg(tf_conv.fromMatrix(camera_trans))

		# print marker.pose.pose
		# print arm_pose.pose
		# print scipy_distance(new_marker, pose)



if __name__ == '__main__':
	try:
		rospy.init_node('calibrator', anonymous=True)
		CalibrateConverter()
		# move_robot()
		rospy.spin()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"