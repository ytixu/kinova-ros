#! /usr/bin/env python
import rospy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

import message_filters
from sensor_msgs.msg import JointState
from moveit_msgs.msg import CollisionObject
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import GetLinkState

links =  ['j2s7s300::root', 'j2s7s300::j2s7s300_link_1', 'j2s7s300::j2s7s300_link_2', 'j2s7s300::j2s7s300_link_3', 'j2s7s300::j2s7s300_link_4', 'j2s7s300::j2s7s300_link_5', 'j2s7s300::j2s7s300_link_6', 'j2s7s300::j2s7s300_link_7', 'j2s7s300::j2s7s300_link_finger_1', 'j2s7s300::j2s7s300_link_finger_2', 'j2s7s300::j2s7s300_link_finger_3']
THRSH = 0.0

class robot_recorder:
	def __init__(self):
		self.header = []
		# self.record_obstacles = None
		self.record_pose = np.zeros((len(links), 3))
		self.count = 0

		self.model_info_prox = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
		# print 'Waiting for service... '
		rospy.wait_for_service('/gazebo/get_link_state')
		# print 'Done waiting for service!'
		# for link in links:
		# 	print link, ": " , model_info_prox(link, 'world')

		self.joint_state_sub = rospy.Subscriber('/j2s7s300/joint_states', JointState, self.record_messages)
		# self.obstacle_sub = rospy.Subscriber('/collision_object', CollisionObject, self.update_obstacles)


	# def update_obstacles(self, obstacle):
	# 	if obstacle.id != 'target_cylinder':
	# 		return
	# 	self.record_obstacles = obstacle

	def record_messages(self, joint_states):
		states = {}
		for link in links:
			js = self.model_info_prox(link, 'world')
			states[link] = [js.link_state.pose.position.x, js.link_state.pose.position.y, js.link_state.pose.position.z,
							js.link_state.pose.orientation.x, js.link_state.pose.orientation.y, js.link_state.pose.orientation.z]

		pose_mat = np.array([js[:3] for js in states.values()])
		if np.linalg.norm(np.round(pose_mat) - np.round(self.record_pose)) > THRSH:
			self.record_pose = pose_mat
			self.count += 1
			# print self.count
			self.format(joint_states, states)

	def format(self, joint_states, link_state):
		# if self.record_obstacles is None:
		# 	return

		if len(self.header) == 0:
			self.header = joint_states.name + \
					[js + '-velocity' for js in joint_states.name] + \
					[js + '-effort' for js in joint_states.name]

			for ls in link_state.keys():
				self.header = self.header + \
					[ls+'-x', ls+'-y', ls+'-z', ls+'-ox', ls+'-oy', ls+'oz']
			# self.header = self.header + \
			# 		['object-height', 'object-radius', 'object-pos-x', 'object-pos-y', 'object-pos-z']

			print '\t'.join(self.header)

		line = list(joint_states.position) + list(joint_states.velocity) + list(joint_states.effort)
		for ls in link_state.values():
			line = line + ls

		# line = line + [self.record_obstacles.primitives[0].dimensions[0],
		# 		self.record_obstacles.primitives[0].dimensions[1],
		# 		self.record_obstacles.primitive_poses[0].position.x,
		# 		self.record_obstacles.primitive_poses[0].position.y,
		# 		self.record_obstacles.primitive_poses[0].position.z]
		print '\t'.join(map(str, line))
		# print len(line), len(self.header)

if __name__ == '__main__':
	rospy.init_node('skeleton_constructor', anonymous=True)
	rr = robot_recorder()
	rospy.spin()

# rosbag record /collision_object /gazebo/link_states /j2s7s300/joint_states