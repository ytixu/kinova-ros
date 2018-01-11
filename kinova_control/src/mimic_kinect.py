#! /usr/bin/env python

import sys
import rospy
import numpy as np
from skeleton_markers.msg import Skeleton
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf


# number of frames before mimicing an action
STEP_DIFF = 5
# min difference for putting in waypoint
MIN_DIFF = 0.2
# min confidence
MIN_CONF = 0.9
# reach distance of the kinova arm
K_REACH = 0.985
# minimum height to avoid touching the table
LOW_REACH = 0.1
LOW_GRAB = 0.03

KINECT_KEYS = ['right_hip', 'right_shoulder', 'right_elbow', 'right_hand']

def get_commander():
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	arm_group = moveit_commander.MoveGroupCommander('arm')
	gripper_group = moveit_commander.MoveGroupCommander('gripper')
	return robot, arm_group, gripper_group

def plan_and_move(group, coords):
	try:
		group.set_joint_value_target(coords)
		plan = group.plan()
		print '---- Success', coords
		group.execute(plan)
		return True

	except (moveit_commander.exception.MoveItCommanderException):
		print 'Failed to plan', coords
		return False

class BadArmException(Exception):
    def __init___(self,dErrorArguments):
        Exception.__init__(self,"Bad arm {0}".format(dErrArguments))
        self.dErrorArguments = dErrorArguements

class KinovaKinectImitator:

	def __init__(self):
		self.prev_pose = []
		self.prev_waypoint = None
		self.gripper_open = False

		self.robot, self.arm_group, self.gripper_group = get_commander()
		self.move_gripper(1.2)
		self.sub = rospy.Subscriber('/skeleton', Skeleton, self.initialize)

		self.key_idx = None

		# for statistics
		self.bad_pose_count = 0

	def initialize(self, msg):
		self.key_idx = [msg.name.index(k) for k in KINECT_KEYS]
		self.sub.unregister()
		self.sub = rospy.Subscriber('/skeleton', Skeleton, self.mimic_skeleton)

	def move_gripper(self, state):
		self.gripper_group.clear_pose_targets()
		group_variable_values = [state]*3
		return plan_and_move(self.gripper_group, group_variable_values)

	def open_gripper(self):
		if self.gripper_open:
			return True

		self.gripper_open = True
		return self.move_gripper(0)

	def close_gripper(self):
		if self.gripper_open:
			self.gripper_open = False
			return self.move_gripper(1.2)

		return True

	def normalize_and_standardize(self, kinect_pose):
		# arm is lower than hip or too close to the base
		if (kinect_pose[1,-1] < 0) or (np.linalg.norm(kinect_pose[:,0]-kinect_pose[:,-1]) < 0.1):
			raise BadArmException(kinect_pose)

		if kinect_pose[1,-1] < LOW_REACH:
			# self.open_gripper()
			kinect_pose[1,-1] = max(LOW_GRAB, kinect_pose[1,-1])

		arm_l = np.sum([np.linalg.norm(kinect_pose[:,i]-kinect_pose[:,i+1]) for i in range(len(kinect_pose[0])-1)])
		return kinect_pose / arm_l * K_REACH

	def format_pose_from_posemsg(self, pose_msg):
		human_pose = np.array([
			[p.x for p in pose_msg],
			[p.y for p in pose_msg],
			[p.z for p in pose_msg]])

		human_pose[0,:] = -(human_pose[0,:] - human_pose[0,0])
		human_pose[1,:] = human_pose[1,:] - human_pose[1,0]
		human_pose[2,:] = human_pose[2,:] - human_pose[2,0]

		return self.normalize_and_standardize(human_pose)

	def plan_and_execute(self, human_pose, orient=None):
		waypoints = None
		if self.prev_waypoint is None:
			waypoints = [self.arm_group.get_current_pose().pose, None]
		else:
			waypoints = [self.prev_waypoint, None]

		wpose = geometry_msgs.msg.Pose()
		if orient is None:
			o = tf.quaternion_about_axis(np.pi, [1,0,0])
			wpose.orientation.x = o[0]
			wpose.orientation.y = o[1]
			wpose.orientation.z = o[2]
			wpose.orientation.w = o[3]
		else:
			wpose.orientation = orient

		wpose.position.x = human_pose[0,-1]
		wpose.position.y = human_pose[2,-1]
		wpose.position.z = human_pose[1,-1]
		waypoints[1] = wpose

		(plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
		if fraction == 1.0:
			self.prev_pose = human_pose
			self.prev_waypoint = waypoints[1]
			self.arm_group.execute(plan)
		else:
			raise BadArmException(human_pose)

	def update_pose(self, raw_pose, raw_orientation):
		try:
			human_pose = self.format_pose_from_posemsg(raw_pose)
			if (len(self.prev_pose) == 0) or (np.linalg.norm(self.prev_pose - human_pose) > MIN_DIFF):
				self.plan_and_execute(human_pose)

		except(BadArmException):
			self.bad_pose_count += 1

	def mimic_skeleton(self, msg):
		arm = [msg.position[i] for i in self.key_idx]
		gripper = msg.orientation[self.key_idx[-1]]
		confidence = np.prod([msg.confidence[i] for i in self.key_idx])

		if confidence > MIN_CONF:
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
