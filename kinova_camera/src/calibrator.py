#!/usr/bin/env python
# Convert ARMarker and KinovaPose into geometry_msgs/Transform

import rospy
import copy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PoseStamped
from visp_hand2eye_calibration.msg import TransformArray
from ar_pose.msg import ARMarker

# from kinova_demo.robot_control_modules import *

# move robot
prefix = 'j2n6s300_'
nbJoints = 6

positions = [
	[2.331199658097515, 3.7644775367356633, 1.5025293880440236, -2.6083915843651044, 7.700129752536169, 8.083529075708338, 0.0, 0.0, -0.0012319970456220702],
	[3.4152468366653213, 3.2943379915951785, 0.6149818194530053, -2.446951291716719, 8.421387278975713, 7.970786814120922, 0.0, 0.0, -0.0012319970456220702],
	[3.3660553218543585, 4.858735541238942, 0.927675280948005, -2.2955164952837617, 8.510542455420515, 9.193059510114892, 0.0, 0.0, -0.0012319970456220702],
	[3.2311851210855846, 4.569246202321803, 1.2734754300601614, -2.9608441767222238, 8.622286031599149, 8.44322306938549, 0.0, 0.0, -0.0012319970456220702],
	[3.0846931516358143, 4.9418655820335475, 0.9101402293825962, -2.0732871539093365, 8.3234628456935, 9.384614295737677, 0.0, 0.0, -0.0012319970456220702],
	[2.29241098313662, 4.566084497475661, 1.282641764216105, -2.9633430207730456, 8.616144782125426, 8.434771261350015, 0.0, 0.0, -0.0012319970456220702],
	[2.682820278930094, 4.6790078540172, 2.22907728398901, -2.7556934211023685, 7.813392928432803, 8.226599415789574, 0.0, 0.0, -0.0012319970456220702],
	[2.6634298030332273, 4.061023187277076, 2.102828934091313, -2.4358717426336627, 7.415752617647546, 7.933924403576482, 0.0, 0.0, -0.0012319970456220702],
	[1.9250496155230732, 3.626548695454933, 1.1718883513307616, -2.6425271842694134, 7.9397876190324, 8.096926906520308, 0.0, 0.0, -0.0012319970456220702],
	[3.0735922972640366, 3.0635625662826897, 1.355879226825502, -2.2326054398011466, 7.6503707180969105, 7.461381621868309, 0.0, 0.0, -0.0012319970456220702]
]


def move_robot():
	for pos in positions:
		raw_input('Next movement.')
		result = joint_position_client(pos, prefix)


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
		# move_robot()
		CalibrateConverter()
		rospy.spin()
	except rospy.ROSInterruptException:
		print "program interrupted before completion"