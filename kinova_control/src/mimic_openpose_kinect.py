#! /usr/bin/env python

import sys
import rospy
import json
import numpy as np
from sklearn.metrics.pairwise import cosine_similarity as scipy_cos_similarity
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D

# number of frames before mimicing an action
STEP_DIFF = 5
# min difference for putting in waypoint
MIN_DIFF = 0.0
# reach distance of the kinova arm
K_REACH = 0.985
# minimum height to avoid touching the table
LOW_REACH = 0.1
LOW_GRAB = 0.03

OPP_FILE = '/home/ytixu/gitHTML/whileAlive/robotics/pose2vec/openpose_utk_pose.json'
UTK_FILE = '/home/ytixu/gitHTML/whileAlive/robotics/pose2vec/utk_pose.json'
UTK_IND_FILE = '/home/ytixu/ROS_pack/forked_arm/data/utk_actions.txt'

OPP_KEYS = ['R_Hip', 'R_Shoulder', 'R_Elbow', 'R_Wrist']
UTK_KEYS = ['L_Hip', 'L_Shoulder', 'L_Elbow', 'L_Wrist', 'L_Hand']

def angles(a,b):
	cos_sim = [scipy_cos_similarity([a[i], a[j]], [b[i], b[j]]) for i,j in [(0,1),(1,2),(0,2)]]
	print cos_sim
	return np.arccos(cos_sim).flatten() * np.sign([a[0], a[2], 1])

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

def generate_and_plot_3D(u_pose, u_pose_raw, o_pose=np.array([])):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	l = len(u_pose_raw)/3
	xs = u_pose_raw[:l]
	ys = u_pose_raw[-l:]
	zs = u_pose_raw[l:-l]
	ax.scatter(xs, ys, zs, color='b', s=2)
	xs = u_pose[0]
	ys = u_pose[2]
	zs = u_pose[1]
	ax.plot(xs, ys, zs, color='r')

	if o_pose.any():
		xs = o_pose[0]
		ys = [0]* (len(xs))
		zs = o_pose[1]
		ax.scatter(xs, ys, zs, color='b')

	ax.set_xlim(-1, 1)
	ax.set_ylim(-1, 1)
	ax.set_zlim(-1, 1)

	plt.show()

# pose should be formated like
# [[x1,x2,x3,...], [y1,y1,y2,...], [z1,z2,z3,...]]
def iter_poses():
	# opp_poses = json.load(open(OPP_FILE, 'r'))
	u_ids = None
	with open(UTK_IND_FILE, 'r') as utk_file:
		u_ids = [utk_pose.split('   ')[0] for utk_pose in utk_file.readlines()]

	utk_poses = json.load(open(UTK_FILE, 'r'))
	utk_header_idx = [utk_poses['header'].index(i) for i in UTK_KEYS]
	# for k, u_pose in utk_poses.iteritems():
	# 	if k not in ['header', 'dimension']:
	for k in u_ids:
		u_pose = utk_poses['s08_e01__'+k]
		utk_l = len(u_pose)/3
		yield [[u_pose[utk_l*j + i] for i in utk_header_idx] for j in range(3)], u_pose, None, None


class BadArmException(Exception):
    def __init___(self,dErrorArguments):
        Exception.__init__(self,"Bad arm {0}".format(dErrArguments))
        self.dErrorArguments = dErrorArguements

class KinovaImitator:

	def __init__(self):
		self.prev_pose = []
		self.prev_waypoint = None
		self.gripper_open = False
		self.key_idx = None

		self.robot, self.arm_group, self.gripper_group = get_commander()
		self.move_gripper(1.2)

		# for statistics
		self.bad_pose_count = 0

		self.mimic_start()

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

	def format_pose(self, human_pose):
		human_pose[0,:] = -(human_pose[0,:] - human_pose[0,0])
		human_pose[1,:] = human_pose[1,:] - human_pose[1,0]
		human_pose[2,:] = human_pose[2,:] - human_pose[2,0]

		return self.normalize_and_standardize(human_pose)

	def plan_and_execute(self, human_pose):
		waypoints = None
		if self.prev_waypoint is None:
			waypoints = [self.arm_group.get_current_pose().pose, None]
		else:
			waypoints = [self.prev_waypoint, None]

		wpose = geometry_msgs.msg.Pose()
		# if human_pose[-1,1] < human_pose[1,1]:
		# 	o = tf.quaternion_about_axis(np.pi, [1,0,0])
		# 	wpose.orientation.x = o[0]
		# 	wpose.orientation.y = o[1]
		# 	wpose.orientation.z = o[2]
		# 	wpose.orientation.w = o[3]
		# else:
		orient = angles(human_pose[:,-1]-human_pose[:,-2], [0,1,0])
		print orient
		orient = tf.quaternion_from_euler(orient[0], orient[1], orient[2])
		wpose.orientation.x = orient[0]
		wpose.orientation.y = orient[1]
		wpose.orientation.z = orient[2]
		wpose.orientation.w = orient[3]

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

	def update_pose(self, raw_pose):
		human_pose = self.format_pose(raw_pose)
		if (len(self.prev_pose) == 0) or (np.linalg.norm(self.prev_pose - human_pose) > MIN_DIFF):
			self.plan_and_execute(human_pose)

	def mimic_start(self):
		for u_pose, u_pose_raw, o_pose, o_pose_raw in iter_poses():
			# print u_pose
			arm = np.array(u_pose)
			try:
				self.update_pose(arm)
				generate_and_plot_3D(np.array(u_pose), u_pose_raw)
			except BadArmException as e:
				self.bad_pose_count += 1
				print 'BAD POSES', self.bad_pose_count
				print e


if __name__ == '__main__':
	# roslaunch openni_launch openni.launch
	# roslaunch skeleton_markers skeleton.launch
	try:
		rospy.init_node('mimic')
		KinovaImitator()
		# print angles([1,1,1], [0,1,0])
		rospy.spin()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
