#! /usr/bin/env python

import sys
import numpy as np
import moveit_commander
import rospy
import tf.transformations as tf
from sklearn.metrics.pairwise import cosine_similarity as scipy_cos_similarity
from std_msgs.msg import Header
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint
from geometry_msgs.msg import Pose, PoseStamped, PointStamped

from move_queue import MoveQueue

# number of frames before mimicing an action
STEP_DIFF = 5
# min difference for putting in waypoint
MIN_DIFF = 0.2
# reach distance of the kinova arm
K_REACH = 0.985
# minimum height to avoid touching the table
LOW_REACH = 0.1
LOW_GRAB = 0.05
MIN_DIST = 0.1
MAX_DIST = 0.5
BASE_HEIGHT = 0.2755*3
RAND_POSE_N = 1
ARM_INIT_POSE = np.array([0,0,1])


def end_effector_orientation_constraint(group, quad):
	const = Constraints()
	const.name = 'angle_constraint'
	const.orientation_constraints = [OrientationConstraint()]
	const.orientation_constraints[0].header = Header()
	const.orientation_constraints[0].header.frame_id = group.get_pose_reference_frame()
	const.orientation_constraints[0].link_name = group.get_end_effector_link()
	const.orientation_constraints[0].orientation.x = quad[0]
	const.orientation_constraints[0].orientation.y = quad[1]
	const.orientation_constraints[0].orientation.z = quad[2]
	const.orientation_constraints[0].orientation.w = quad[3]
	const.orientation_constraints[0].absolute_x_axis_tolerance = 0.5
	const.orientation_constraints[0].absolute_y_axis_tolerance = 0.5
	const.orientation_constraints[0].absolute_z_axis_tolerance = 0.5
	const.orientation_constraints[0].weight = 1.0
	return const

def shoulder_constraint(group):
	const = Constraints()
	const.name = 'right_shoulder'
	const.joint_constraints = [JointConstraint()]
	const.joint_constraints[0].joint_name = 'j2s7s300_joint_2'
	const.joint_constraints[0].position = np.pi*3/4
	const.joint_constraints[0].tolerance_above = np.pi/4
	const.joint_constraints[0].tolerance_below = np.pi/4
	const.joint_constraints[0].weight = 1.0
	return const

def joint_position_constraint(group, coords):
	const = Constraints()
	const.name = 'joint_pose_constraint'
	const.joint_constraints = [None]*6
	curr_val = group.get_current_joint_values()
	for i in range(6):
		const.joint_constraints[i] = JointConstraint()
		const.joint_constraints[i].joint_name = 'j2s7s300_joint_%d'%(i+1)
		const.joint_constraints[i].position = curr_val[i]
		const.joint_constraints[i].tolerance_above = 0.4
		const.joint_constraints[i].tolerance_below = 0.4
		const.joint_constraints[i].weight = 1.0

	return const

def plan_and_move(group, coords, angle=True):
	try:
		if angle:
			group.set_joint_value_target(coords)
		else:
			group.set_position_target(coords[:3])
			# group.set_pose_target(coords)

		plan = group.plan()
		return group.execute(plan)

	except (moveit_commander.exception.MoveItCommanderException):
		return False

def get_quad(a, b, base_vec):
	to_vec = np.array(a-b)[[0,2,1]]
	axis = np.cross(base_vec, to_vec)
	angle = np.arccos(np.dot(base_vec, to_vec))
	return tf.quaternion_about_axis(angle, axis)

class BadArmException(Exception):
    def __init___(self,dErrorArguments):
        Exception.__init__(self,"Bad arm {0}".format(dErrArguments))
        self.dErrorArguments = dErrorArguements


class MessageCommander:

	def __init__(self, close_gripper=True):
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		self.arm_group = moveit_commander.MoveGroupCommander('arm')
		self.gripper_group = moveit_commander.MoveGroupCommander('gripper')
		self.prev_pose = []
		if close_gripper:
			self.move_gripper(1.2)

		self.pose_pub = rospy.Publisher('mimic_pose', PoseStamped, queue_size=10)
		self.hand_pub = rospy.Publisher('mimic_hand', PointStamped, queue_size=10)
		self.elbow_pub = rospy.Publisher('mimic_elbow', PointStamped, queue_size=10)

		self.move_queue = MoveQueue()
		self.fails = 0
		# self.arm_group.set_path_constraints(shoulder_constraint(self.arm_group))


	def format_point(self, coords):
		ppoint = PointStamped()
		ppoint.header.stamp = rospy.Time.now()
		ppoint.header.frame_id = self.arm_group.get_pose_reference_frame()
		ppoint.point.x = coords[0]
		ppoint.point.y = coords[1]
		ppoint.point.z = coords[2]
		return ppoint

	def publish_elbow(self, coords):
		self.elbow_pub.publish(self.format_point(coords))

	def publish_hand(self, coords):
		self.hand_pub.publish(self.format_point(coords))

	def publish_pose(self, pose):
		ppose = PoseStamped()
		ppose.header.stamp = rospy.Time.now()
		ppose.header.frame_id = self.arm_group.get_pose_reference_frame()
		ppose.pose.position.x = pose[0]
		ppose.pose.position.y = pose[1]
		ppose.pose.position.z = pose[2]
		ppose.pose.orientation.x = pose[3]
		ppose.pose.orientation.y = pose[4]
		ppose.pose.orientation.z = pose[5]
		ppose.pose.orientation.w = pose[6]
		self.pose_pub.publish(ppose)

	def move_gripper(self, state):
		self.gripper_group.clear_pose_targets()
		group_variable_values = [state]*3
		return plan_and_move(self.gripper_group, group_variable_values)

	def format_message(self, arm_coord, hand_coord=None):
		arm_coord[0,:] = -(arm_coord[0,:] - arm_coord[0,0])
		arm_coord[1,:] = arm_coord[1,:] - arm_coord[1,0]
		arm_coord[2,:] = arm_coord[2,:] - arm_coord[2,0]

		# arm is lower than hip or too close to the base
		if np.linalg.norm(arm_coord[:,0]-arm_coord[:,-1]) < 0.1:
			raise BadArmException(arm_coord)

		# arm_coord_ = np.copy(arm_coord)
		arm_coord[1,1:] = arm_coord[1,1:] - arm_coord[1,1] + BASE_HEIGHT
		arm_coord[2,1:] = arm_coord[2,1:] - 0.2
		arm_l = np.sum([np.linalg.norm(arm_coord[:,i]-arm_coord[:,i+1]) for i in range(len(arm_coord[0])-1)])
		arm_coord = arm_coord / arm_l * K_REACH

		if arm_coord[1,-1] < LOW_REACH:
			arm_coord[1,-1] = max(LOW_GRAB, arm_coord[1,-1])

		# angle
		orient = None
		if (np.linalg.norm(arm_coord[:,0]-arm_coord[:,-1]) < MAX_DIST):
			orient = tf.quaternion_about_axis(np.pi, [1,0,0])
		# 	print 'using virtical pose'
		else:
			orient = get_quad(arm_coord[:,-1], arm_coord[:,-2], ARM_INIT_POSE)
		# self.arm_group.set_path_constraints(end_effector_orientation_constraint(self.arm_group, orient))

		pose = np.zeros(7)
		pose[:3] = arm_coord[[0,2,1],-1]
		pose[3:] = orient
		return pose, arm_coord[[0,2,1],-2]

	def set_message(self, arm_coord, hand_coord=None):
		new_pose, elbow = self.format_message(arm_coord)
		self.publish_pose(new_pose)
		# if (len(self.prev_pose) == 0) or (diff > MIN_DIFF):
		self.arm_group.clear_path_constraints()
		if len(self.prev_pose) != 0:
			diff = np.linalg.norm(self.prev_pose - new_pose)
			if diff < MIN_DIFF:
				return
			# if diff < 0.4:
			if self.fails < 5:
				self.arm_group.set_path_constraints(joint_position_constraint(self.arm_group, new_pose))
			# elif diff > 0.4:
			else:
				self.move_queue.record()
		# 	self.arm_group.set_path_constraints(end_effector_orientation_constraint(self.arm_group, new_pose[3:]))

		# print self.arm_group.get_path_constraints()
		self.publish_elbow(elbow)
		self.publish_hand(new_pose[:3])
		self.send(new_pose, arm_coord)

	# def send_sequence(self):
	# 	self.prev_pose == 0
	# 	self.arm_group.clear_path_constraints()
	# 	move_queue, n, _= self.move_queue.get()
	# 	for i in range(n):
	# 		if plan_and_move(self.arm_group, move_queue[i]['r']):
	# 			print 'success'
	# 		else:
	# 			print 'failed'
		# targets = np.zeros((n+1,7))
		# targets[1:] = [move_queue[i]['r'] for i in range(n)]
		# targets[0] = targets[-1:]
		# print targets
		# self.arm_group.set_pose_targets(targets.tolist())
		# try:
		# 	plan = self.arm_group.plan()
		# 	return self.arm_group.execute(plan)
		# except (moveit_commander.exception.MoveItCommanderException):
		# 	return False

		# waypoints = [None]*(n)
		# waypoints[0] = self.arm_group.get_current_pose().pose
		# for i in range(1,n):
		# 	waypoints[i] = Pose()
		# 	waypoints[i].position.x = move_queue[i]['r'][0]
		# 	waypoints[i].position.y = move_queue[i]['r'][1]
		# 	waypoints[i].position.z = move_queue[i]['r'][2]
		# 	waypoints[i].orientation.x = move_queue[i]['r'][3]
		# 	waypoints[i].orientation.y = move_queue[i]['r'][4]
		# 	waypoints[i].orientation.z = move_queue[i]['r'][5]
		# 	waypoints[i].orientation.w = move_queue[i]['r'][6]

		# (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
		# if fraction == 1.0:
		# 	self.arm_group.execute(plan)
		# else:
		# 	raise BadArmException(move_queue)

	def send(self, pose, h_pose):
		# self.prev_pose = pose
		current_joint_val = np.array(self.arm_group.get_current_joint_values())
		if plan_and_move(self.arm_group, pose.tolist(), False):
			diff = current_joint_val - np.array(self.arm_group.get_current_joint_values())
			print np.max(diff), np.average(diff)
			self.prev_pose = pose

			self.move_queue.push(self.arm_group.get_current_joint_values(), h_pose.tolist())
			self.move_queue.record(True)
				# self.send_sequence()
			self.fails = 0
			return
			# plan_and_move(self.arm_group, self.arm_group.get_random_pose(), False)

		self.fails += 1
		raise BadArmException(pose)
