#! /usr/bin/env python

import sys
import rospy
import numpy as np
from skeleton_markers.msg import Skeleton
from utils.message_commander import MessageCommander, BadArmException
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def generate_and_plot_3D(coords, projects=None):
	xs = coords[0,:]
	ys = coords[2,:]
	zs = coords[1,:]

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(xs, ys, zs)

	# xs = [x[0][0] for x in projects]
	# ys = [1 for i in range(len(projects))]
	# zs = [x[0][1] for x in projects]

	# ax.plot(xs, ys, zs)

	plt.show()

MIN_CONF = 0.9

KINECT_KEYS = ['right_hip', 'right_shoulder', 'right_elbow', 'right_hand']
# KINECT_KEYS = [
# 'head',
# 'neck',
# 'torso',
# 'left_shoulder',
# 'left_elbow',
# 'left_hand',
# 'left_hip',
# 'left_knee',
# 'left_foot',
# 'right_shoulder',
# 'right_elbow',
# 'right_hand',
# 'right_hip',
# 'right_knee',
# 'right_foot']

class KinovaKinectImitator:

	def __init__(self):
		self.message_commander = MessageCommander()
		self.sub = rospy.Subscriber('/skeleton', Skeleton, self.initialize)

		self.key_idx = None

		# for statistics
		self.bad_pose_count = 0

	def initialize(self, msg):
		self.key_idx = [msg.name.index(k) for k in KINECT_KEYS]
		self.sub.unregister()
		self.sub = rospy.Subscriber('/skeleton', Skeleton, self.mimic_skeleton)


	def format_pose_from_posemsg(self, pose_msg):
		human_pose = np.array([
			[p.x for p in pose_msg],
			[p.y for p in pose_msg],
			[p.z for p in pose_msg]])

		return human_pose

	# def plan_and_execute(self, human_pose):
	# 	waypoints = None
	# 	if self.prev_waypoint is None:
	# 		waypoints = [self.arm_group.get_current_pose().pose, None]
	# 	else:
	# 		waypoints = [self.prev_waypoint, None]

	# 	wpose = Pose()
	# 	print human_pose[1,-1]
	# 	if human_pose[1,-1] < LOW_REACH:
	# 		o = tf.quaternion_about_axis(np.pi, [1,0,0])
	# 		wpose.orientation.x = o[0]
	# 		wpose.orientation.y = o[1]
	# 		wpose.orientation.z = o[2]
	# 		wpose.orientation.w = o[3]
	# 	else:
	# 		orient = angles(human_pose[:,-1]-human_pose[:,-2], [0,1,0])
	# 		print orient
	# 		orient = tf.quaternion_from_euler(orient[0], orient[1], orient[2])
	# 		wpose.orientation.x = orient[0]
	# 		wpose.orientation.y = orient[1]
	# 		wpose.orientation.z = orient[2]
	# 		wpose.orientation.w = orient[3]

	# 	wpose.position.x = human_pose[0,-1]
	# 	wpose.position.y = human_pose[2,-1]
	# 	wpose.position.z = human_pose[1,-1]
	# 	waypoints[1] = wpose

	# 	(plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
	# 	if fraction == 1.0:
	# 		self.prev_pose = human_pose
	# 		self.prev_waypoint = waypoints[1]
	# 		# print self.arm_group.get_path_constraints()
	# 		self.arm_group.execute(plan)
	# 	else:
	# 		raise BadArmException(human_pose)

	def update_pose(self, raw_pose, raw_orientation):
		human_pose = self.format_pose_from_posemsg(raw_pose)
		# generate_and_plot_3D(human_pose)
		try:
			self.message_commander.set_message(human_pose)
		except BadArmException as e:
			self.bad_pose_count += 1
			print 'BAD ARM', self.bad_pose_count#, e

	def mimic_skeleton(self, msg):
		arm = [msg.position[i] for i in self.key_idx]
		gripper = msg.orientation[self.key_idx[-1]]
		confidence = np.prod([msg.confidence[i] for i in self.key_idx])

		if confidence > MIN_CONF:
			# print confidence
			self.update_pose(arm, gripper)


if __name__ == '__main__':
	# roslaunch openni_launch openni.launch
	# roslaunch skeleton_markers skeleton.launch
	try:
		rospy.init_node('move_robot_kinect_pose')
		KinovaKinectImitator()
		rospy.spin()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
