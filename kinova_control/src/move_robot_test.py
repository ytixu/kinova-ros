#! /usr/bin/env python
""" Test trajectories """

import rospy
import tf.transformations as tf
import math
import sys
import numpy as np
from move_robot import moveJoint, moveFingers
from skeleton_markers.msg import Skeleton
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg



def test_move():
	# position in rad
	joint_tests = [
		[1.0,2.9,1.3,4.2,1.4,0.0],
		[0.0,1.9,1.3,4.2,1.4,0.0],
		[0.0,2.9,2,4.2,1.4,0.0],
		[0.0,2.9,1.3,5.2,1.4,0.0],
		[0.0,2.9,1.3,4.2,0,0.0],
		[0.0,2.9,1.3,4.2,1.4,1.0]
	]

	# positions in degree
	positions = [
		[-3.260101318359375, 283.1480407714844, 52.1471939086914, -118.79060363769531, 116.89929199218756, 177.69879150390625, 1.0],
		[-46.432098388671875, 215.6886749267578, 86.08859252929689, -149.4498291015625, 103.15209960937506, 81.1849365234375, 0.0],
		[15.679229736328125, 188.75166320800778, 35.235862731933594, -140.19998168945312, 122.50994873046875,  96.692443847656255, 0.0],
		[5.133270263671875, 261.79852294921875, 72.9647674560547, -169.64387512207028, 134.02059936523438, 123.76104736328125, 0.0],
		[-48.65452575683594, 261.61737060546875, 73.48995971679688, -169.78704833984375, 133.66873168945312, 123.27679443359375, 0.0],
		[-26.285720825195312, 268.08740234375, 127.7167205810547, -157.8896026611328, 87.6744384765625, 111.34942626953125, 0.0],
		[-27.396713256835938, 232.6794891357422, 120.48322296142578, -139.56517028808594, 64.89132690429688, 94.58038330078125, 0.0],
		[-69.396713256835938, 232.6794891357422, 120.48322296142578, -139.56517028808594, 64.89132690429688, 94.58038330078125, 0.0],
		[-3.896133422851591, 175.52920532226565, 77.6861572265625, -127.91886901855469, 78.33395385742188, 67.5056762695312, 0.0],
	]


	# rosrun kinova_demo joints_action_client.py -v radian -- 1.0 2.9 1.3 4.2 1.4 0.0

	finger_tests = [
		[1.2,1.2, 1],
		[0,0,1],
		[1.2,1.2,1.2]
	]

	rospy.init_node('move_robot_using_trajectory_msg_test')
	prefix = 'j2n6s300'
	nbJoints = 6
	nbfingers = 3

	# go to home position first
	moveJoint ([0.0,2.9,1.0,4.2,1.5,1.3],prefix,nbJoints)
	moveFingers ([0,0,0],prefix,nbfingers)

	# TEST HERE

	print 'Test all joints.'
	for test in positions:
		test = [math.pi / 180.0 * i for i in test]
		print test
		raw_input("Press Enter to continue")
		count = 0
		while (count < 5):
			count = count + 1
			moveJoint (test,prefix,nbJoints)

	# print 'Test fingers.'
	# for test in finger_tests:
	#   raw_input("Press Enter to continue...")
	#   moveFingers (test,prefix,nbfingers)

	print 'DONE!'

class pose_getter:
	def __init__(self):
		from sensor_msgs.msg import JointState
		self.coord = []
		rospy.Subscriber('/j2s7s300/joint_states', JointState, self.record_messages)

	def record_messages(self, joint_states):
		self.coord = np.array(list(joint_states.position))[:-3]%(2*np.pi)

def get_cartesian_pose(robot, group):
	group.clear_pose_targets()
	print group.get_current_pose().pose
	rospy.sleep(5)
	# waypoints = [group.get_current_pose().pose, None]
	# wpose = geometry_msgs.msg.Pose()
	# wpose.orientation.w = np.random.uniform(-1,1,1)[0]
	# wpose.orientation.x = np.random.uniform(-1,1,1)[0]
	# wpose.orientation.y = np.random.uniform(-1,1,1)[0]
	# wpose.orientation.z = np.random.uniform(-1,1,1)[0]
	# wpose.position.x = np.random.uniform(-1.5,1.5,1)[0]
	# wpose.position.y = np.random.uniform(-1.5,1.5,1)[0]
	# wpose.position.z = np.random.uniform(0.05,1.5,1)[0]
	# waypoints[1] = wpose
	# try:
	#   (plan, fraction) = group.compute_cartesian_path(
	#                            waypoints,   # waypoints to follow
	#                            0.01,        # eef_step
	#                            0.0)         # jump_threshold
	#   print '----Success', wpose
	#   group.execute(plan)
	#   rospy.sleep(5)
	#   return True

	# except (moveit_commander.exception.MoveItCommanderException):
	#   print 'Failed to plan', wpose
	#   return False


def plan_and_move(group, coords):
	try:
		group.set_joint_value_target(coords)
		plan = group.plan()
		rospy.sleep(2)
		print '---- Success', coords
		group.execute(plan)
		rospy.sleep(4)
		group.stop()
		return True

	except (moveit_commander.exception.MoveItCommanderException):
		print 'Failed to plan', coords
		return False


def get_pose_msg(robot, group, scale, nb):
	group.clear_pose_targets()
	# group_variable_values = group.get_current_joint_values()
	# group_variable_values[np.random.randint(0,nb,1)] = np.random.uniform(0,1,1)[0]*scale
	group_variable_values = np.random.uniform(0,1,nb)*scale
	return plan_and_move(group, group_variable_values)


def get_commander():
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	arm_group = moveit_commander.MoveGroupCommander('arm')
	gripper_group = moveit_commander.MoveGroupCommander('gripper')
	return robot, arm_group, gripper_group


def rand_poses():
	nbfingers = 3
	nbJoints = 7
	prefix = 'j2s7s300'
	rospy.init_node('move_robot_random_trajectory')
	robot, arm_group, gripper_group = get_commander()

	while True:
		# get_cartesian_pose(robot, arm_group)
		if get_pose_msg(robot, arm_group, 2*np.pi,nbJoints):
			moveJoint([0.0,2.9,0.0,1.3,4.2,1.4,0.0],prefix,nbJoints)
			get_pose_msg(robot, gripper_group, 1.2, nbfingers)



# def ik_poses():
#   import ikpy
#   nbfingers = 3
#   nbJoints = 7
#   prefix = 'j2s7s300'
#   rospy.init_node('move_robot_ik')
#   moveit_commander.roscpp_initialize(sys.argv)
#   robot = moveit_commander.RobotCommander()
#   arm_group = moveit_commander.MoveGroupCommander('arm')

#   urdf = '/home/ytixu/gitHTML/whileAlive/robotics/pose2vec/j2s7s300.urdf'
#   arm_chain = ikpy.chain.Chain.from_urdf_file(urdf, base_elements=['j2s7s300_link_base'])
#   # some_pose = arm_chain.forward_kinematics(np.array([0.0,0.0,2.9,0.0,1.3,4.2,1.4,0.0,0.0]))
#   # moveJoint([0.0,2.9,0.0,1.3,4.2,1.4,0.0],prefix,nbJoints)
#   # print some_pose
#   target_vector = [0.5,0.5,0.5]
#   target_frame = np.eye(4)
#   target_frame[:3, 3] = target_vector
#   joint_angles = arm_chain.inverse_kinematics(target_frame)[1:-1]
#   plan_and_move(arm_group, joint_angles)
#   print joint_angles
#   rospy.spin()

def normalize(arm_pose, K_REACH=0.985):
	# arm is lower than hip
	if arm_pose[1,-1] < 0 or np.linalg.norm(arm_pose[:,0]-arm_pose[:,-1]) < 0.1:
		print 'BAD ARM'
		return np.array([])

	arm_l = np.sum([np.linalg.norm(arm_pose[:,i]-arm_pose[:,i+1]) for i in range(len(arm_pose[0])-1)])
	return arm_pose / arm_l * K_REACH

def iter_poses(utk_poses, U_KEYS):
	hip_idx = utk_poses['header'].index(u'R_Hip')
	print utk_poses['header']
	for k, pose in utk_poses.iteritems():
		u_pose = np.reshape(pose, (3,len(pose)/3))
		if k == 'header' or k == 'dimension':
			continue

		arm_pose = np.zeros((3,len(U_KEYS)))
		for i,j in enumerate(U_KEYS):
			arm_pose[:,i] = u_pose[:,utk_poses['header'].index(j)] - u_pose[:,hip_idx]

		yield normalize(arm_pose)

def set_cartesian_pose(arm_group, coords, orient=None, prev_coords=None):
	waypoints = None
	if prev_coords is None:
		waypoints = [arm_group.get_current_pose().pose, None]
	else:
		waypoints = [prev_coords, None]

	wpose = geometry_msgs.msg.Pose()
	if orient is None:
		o = tf.quaternion_about_axis(np.pi, [1,0,0])
		wpose.orientation.x = o[0]
		wpose.orientation.y = o[1]
		wpose.orientation.z = o[2]
		wpose.orientation.w = o[3]
	else:
		wpose.orientation = orient

	wpose.position.x = coords[0,-1]
	wpose.position.y = coords[2,-1]
	wpose.position.z = coords[1,-1]
	waypoints[1] = wpose
	return waypoints, wpose

def set_finger_pose(gripper_group, state):
	gripper_group.clear_pose_targets()
	group_variable_values = [state]*3
	return plan_and_move(gripper_group, group_variable_values)

def mimic():
	nbfingers = 3
	nbJoints = 7
	prefix = 'j2s7s300'
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_robot_mimic')

	import json
	utk_poses = json.load(open('/home/ytixu/gitHTML/whileAlive/robotics/pose2vec/utk_pose.json', 'r'))
	U_KEYS = [u'R_Hip', u'R_Shoulder', u'R_Elbow', u'R_Wrist', u'R_Hand']

	robot, arm_group, gripper_group = get_commander()

	for u_pose in iter_poses(utk_poses, U_KEYS):
		if len(u_pose) != 0:
			waypoints, _ = set_cartesian_pose(arm_group, u_pose)
			(plan, fraction) = arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
			rospy.sleep(4)
			print 'going!!!'
			arm_group.execute(plan)
			rospy.sleep(4)
			arm_group.stop()

class mimicer:
	def __init__(self):
		self.prev_pose = []
		self.robot, self.arm_group, self.gripper_group = get_commander()
		set_finger_pose(self.gripper_group, 1.2)
		rospy.Subscriber('/skeleton', Skeleton, self.mimic_skeleton)
		self.keys = ['right_hip', 'right_shoulder', 'right_elbow', 'right_hand']
		self.key_idx = None
		self.initialized = False
		self.moving = False
		self.prev_waypoint = None

	def update_pose(self, raw_pose, raw_orientation):
		human_pose = np.array([[p.x for p in raw_pose],
			[p.y for p in raw_pose], [p.z for p in raw_pose]])
		# print human_pose
		human_pose[0,:] = -(human_pose[0,:] - human_pose[0,0])
		human_pose[1,:] = human_pose[1,:] - human_pose[1,0]
		human_pose[2,:] = human_pose[2,:] - human_pose[2,0]
		human_pose = normalize(human_pose)
		if (len(self.prev_pose) == 0) and (len(human_pose) == 0):
			return
		if len(human_pose) == 0:
			# self.moving = True
			# self.prev_pose = []
			# moveJoint([0.0,2.9,0.0,1.3,4.2,1.4,0.0],'j2s7s300',7)
			# self.moving = False
			return
		if (len(self.prev_pose) == 0) or (np.linalg.norm(self.prev_pose - human_pose) > 0.2):
			self.moving = True
			self.prev_pose = human_pose
			waypoints, prev_w = set_cartesian_pose(self.arm_group, human_pose, None, self.prev_waypoint)
			self.prev_waypoint = prev_w
			(plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
			self.arm_group.execute(plan)
			# rospy.sleep(4)
			# self.arm_group.stop()
			self.moving = False

	def mimic_skeleton(self, msg):
		if self.moving:
			return
		if not self.initialized:
			self.key_idx = [msg.name.index(k) for k in self.keys]
			self.initialized = True
		arm = [msg.position[i] for i in self.key_idx]
		gripper = msg.orientation[self.key_idx[-1]]
		print gripper
		confidence = np.prod([msg.confidence[i] for i in self.key_idx])
		# print confidence
		if confidence > 0.8:
			# print arm
			self.update_pose(arm, gripper)

def mimic_kinect():
	# roslaunch openni_launch openni.launch
	# roslaunch skeleton_markers skeleton.launch
	rospy.init_node('move_robot_kinect_pose')
	m = mimicer()
	rospy.spin()

if __name__ == '__main__':
	try:
		# rand_poses()
		mimic_kinect()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
