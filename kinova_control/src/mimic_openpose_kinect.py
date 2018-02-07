#! /usr/bin/env python

import sys
import rospy
import json
import numpy as np
import cv2
import copy
from sklearn.metrics.pairwise import cosine_similarity as scipy_cos_similarity
import moveit_commander
from moveit_msgs.msg import Constraints, JointConstraint
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
BASE_HEIGHT = 0.2755
# minimum height to avoid touching the table
LOW_REACH = 0.1
LOW_GRAB = 0.03
MIN_DIST = 0.1
MAX_DIST = 0.5
# number of random pertubations
RAND_PERTUB_N = 5

OPP_FILE = '/home/ytixu/ROS_pack/forked_arm/data/openpose_utk_pose.json'
UTK_FILE = '/home/ytixu/ROS_pack/forked_arm/data/utk_pose.json'
UTK_IND_FILE = '/home/ytixu/ROS_pack/forked_arm/data/utk_actions.txt'
IMAGE_FOLDER = '/media/ytixu/TOSHIBA EXT/robot/openpose/data/RGB/'
IK_URDF = '/home/ytixu/gitHTML/whileAlive/robotics/pose2vec/j2s7s300.urdf'
OUT_DIR = '/home/ytixu/ROS_pack/forked_arm/data/arm_pose/'

OPP_KEYS = ['R_Hip', 'R_Shoulder', 'R_Elbow', 'R_Wrist']
UTK_KEYS = ['R_Hip', 'R_Shoulder', 'R_Elbow', 'R_Wrist', 'R_Hand']

def angles(a,b):
	cos_sim = [scipy_cos_similarity([a[i], a[j]], [b[i], b[j]]) for i,j in [(0,1),(1,2),(0,2)]]
	return np.arccos(cos_sim).flatten() * np.sign([1, a[2], 1])

def get_commander():
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()
	arm_group = moveit_commander.MoveGroupCommander('arm')
	gripper_group = moveit_commander.MoveGroupCommander('gripper')
	return robot, arm_group, gripper_group

def plan_and_move(group, coords, angle=True):
	try:
		if angle:
			group.set_joint_value_target(coords)
		else:
			group.set_pose_target(coords)

		plan = group.plan()
		print '---- Success', coords
		return group.execute(plan)

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

def show_image(k):
	folder, img_file = k.split('__')
	img = cv2.imread(IMAGE_FOLDER+folder+'/colorImg'+img_file+'.jpg')
	cv2.imshow('human', img)

def elbow_up_constraint():
	const = Constraints()
	const.name = 'elbow_up'
	const.joint_constraints = [JointConstraint()]
	const.joint_constraints[0].joint_name = 'j2s7s300_joint_4'
	const.joint_constraints[0].tolerance_above = 3
	const.joint_constraints[0].tolerance_below = 0.5
	return const


# pose should be formated like
# [[x1,x2,x3,...], [y1,y1,y2,...], [z1,z2,z3,...]]
def iter_poses():
	u_ids = None
	# folder = 's08_e01__'
	# with open(UTK_IND_FILE, 'r') as utk_file:
	# 	u_ids = [utk_pose.split('   ')[0] for utk_pose in utk_file.readlines()]

	opp_poses = json.load(open(OPP_FILE, 'r'))
	opp_header_idx = [opp_poses['header'].index(i) for i in OPP_KEYS]
	utk_poses = json.load(open(UTK_FILE, 'r'))
	utk_header_idx = [utk_poses['header'].index(i) for i in UTK_KEYS]

	# for k in u_ids:
	for id_ in opp_poses['poses']:
		u_pose_raw = utk_poses['poses'][id_]
		utk_l = len(u_pose_raw)/3
		u_pose = [[u_pose_raw[utk_l*j + i] for i in utk_header_idx] for j in range(3)]

		o_pose_raw = opp_poses['poses'][id_][u'pose']
		opp_l = len(o_pose_raw)/2
		o_pose = [[o_pose_raw[opp_l*j + i] for i in opp_header_idx] for j in range(2)]

		yield id_, u_pose, u_pose_raw, o_pose, opp_poses['poses'][id_][u'right_hand']


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

		# self.arm_group.set_path_constraints(elbow_up_constraint())

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
		if (np.linalg.norm(kinect_pose[:,0]-kinect_pose[:,-1]) < MIN_DIST):
			raise BadArmException('too close')

		if kinect_pose[1,-1] < LOW_REACH:
			# self.open_gripper()
			kinect_pose[1,-1] = max(LOW_GRAB, kinect_pose[1,-1])

		kinect_pose_ = np.copy(kinect_pose)
		kinect_pose_[1,1:] = kinect_pose_[1,1:] - kinect_pose_[1,1]
		arm_l = np.sum([np.linalg.norm(kinect_pose_[:,i]-kinect_pose_[:,i+1]) for i in range(len(kinect_pose_[0])-1)])
		return kinect_pose / arm_l * K_REACH

	def format_pose(self, human_pose):
		human_pose[0,:] = -(human_pose[0,:] - human_pose[0,0])
		human_pose[1,:] = human_pose[1,:] - human_pose[1,0]
		human_pose[2,:] = human_pose[2,:] - human_pose[2,0]

		return self.normalize_and_standardize(human_pose)


	def plan_and_execute_gripper(self, raw_proj, raw_hand):
		right_hand = np.array(raw_hand)
		confidence = np.prod(right_hand[[i*3+2 for i in [4, 8, 12]]])
		fingers = [right_hand[4*3:4*3+2], right_hand[8*3:8*3+2], right_hand[12*3:12*3+2]]
		n = np.linalg.norm(raw_proj[:,0]-raw_proj[:,1])
		print confidence, np.var(fingers, axis=0)

	def plan_and_execute_arm(self, human_pose, raw_proj, raw_hand):
		orients = []
		pose = np.zeros(7)

		if (human_pose[1,-1] < LOW_REACH) or (np.linalg.norm(human_pose[:,0]-human_pose[:,-1]) < MAX_DIST):
			orients = [tf.quaternion_about_axis(np.pi, [1,0,0])]
			print 'using virtical pose'
		else:
			for vect in [human_pose[:,-2]-human_pose[:,-3], human_pose[:,-2]-np.array([0,BASE_HEIGHT,0])]:
				orient = angles(vect, [0,1,0])
				orients.append(tf.quaternion_from_euler(orient[0], orient[1], orient[2]))
		pose[:3] = human_pose[[0,2,1],-1]

		for orient in orients:
			pose[3:] = orient
			if plan_and_move(self.arm_group, pose.tolist(), False):
				return

		raise BadArmException('failed to find plan')

	def update_pose(self, raw_pose, raw_proj, raw_hand):
		human_pose = self.format_pose(raw_pose)
		if (len(self.prev_pose) == 0) or (np.linalg.norm(self.prev_pose - human_pose) > MIN_DIFF):
			self.plan_and_execute_arm(human_pose, raw_proj, raw_hand)

	def mimic_start(self):
		for k, u_pose, u_pose_raw, o_pose, o_right in iter_poses():
			# print u_pose
			show_image(k)
			if cv2.waitKey(25) & 0xFF == ord('q'):
				raise Exception()
			arm = np.array(u_pose)
			try:
				self.update_pose(arm, np.array(o_pose), o_right)
				json_pose = {
					'key':k,
					'arm': self.arm_group.get_current_joint_values(),
					'gripper': self.gripper_group.get_current_joint_values()
				}
				with open(OUT_DIR+'arm_pose_'+k+'.json', 'w') as outfile:
				    json.dump(json_pose, outfile)

				# generate_and_plot_3D(np.array(u_pose), u_pose_raw)
			except BadArmException as e:
				self.bad_pose_count += 1
				print 'BAD POSES', self.bad_pose_count
				print e
				# break


		cv2.destroyAllWindows()

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
