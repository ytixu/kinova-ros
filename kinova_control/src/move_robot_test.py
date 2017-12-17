#! /usr/bin/env python
""" Test trajectories """

import rospy
import math
import sys
import numpy as np
from move_robot import moveJoint, moveFingers
from sensor_msgs.msg import JointState
import moveit_commander
import moveit_msgs.msg


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
    self.coord = []
    rospy.Subscriber('/j2s7s300/joint_states', JointState, self.record_messages)

  def record_messages(self, joint_states):
    self.coord = np.array(list(joint_states.position))[:-3]%(2*np.pi)

def get_pose_msg(robot, group, scale):
  group.clear_pose_targets()
  group_variable_values = group.get_current_joint_values()
  group_variable_values[np.random.randint(0,7,1)] = np.random.uniform(0,1,1)[0]*scale
  try:
    group.set_joint_value_target(group_variable_values)
    plan = group.plan()
    rospy.sleep(2)
    print '---- Success', group_variable_values
    # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    # display_trajectory.trajectory_start = robot.get_current_state()
    # display_trajectory.trajectory.append(plan)
    return plan

  except (moveit_commander.exception.MoveItCommanderException):
    print 'Failed to plan', group_variable_values
    return None


def rand_poses():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_robot_random_trajectory')

  robot = moveit_commander.RobotCommander()
  arm_group = moveit_commander.MoveGroupCommander('arm')
  gripper_group = moveit_commander.MoveGroupCommander('gripper')
  # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
  #                                 moveit_msgs.msg.DisplayTrajectory, queue_size=10)
  while True:
    tjr = get_pose_msg(robot, arm_group, 2*np.pi)
    if tjr:
      print tjr
      # display_trajectory_publisher.publish(tjr);
      # rospy.sleep(5)
      # gripper_group.go(wait=True)


if __name__ == '__main__':
  try:
    rand_poses()
  except rospy.ROSInterruptException:
    print "program interrupted before completion"
