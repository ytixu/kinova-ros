#! /usr/bin/env python
""" Test trajectories """

import rospy
from move_robot import moveJoint, moveFingers

joint_tests = [
  [1.0,2.9,1.3,4.2,1.4,0.0],
  [0.0,1.9,1.3,4.2,1.4,0.0],
  [0.0,2.9,2,4.2,1.4,0.0],
  [0.0,2.9,1.3,5.2,1.4,0.0],
  [0.0,2.9,1.3,4.2,0,0.0],
  [0.0,2.9,1.3,4.2,1.4,1.0]
]

# rosrun kinova_demo joints_action_client.py -v radian -- 1.0 2.9 1.3 4.2 1.4 0.0

finger_tests = [
  [1.2,1.2, 1],
  [0,0,1],
  [1.2,1.2,1.2]
]

PI = 3.14159265359

if __name__ == '__main__':
  try:
    rospy.init_node('move_robot_using_trajectory_msg_test')
    prefix = 'j2n6s300'
    nbJoints = 6
    nbfingers = 3

    # go to home position first
    moveJoint ([0.0,2.9,1.3,4.2,1.4,0.0],prefix,nbJoints)
    moveFingers ([1,1,1],prefix,nbfingers)

    # TEST HERE

    print 'Test all joints.'

    for test in joint_tests:
      raw_input("Press Enter to continue...")
      moveJoint (test,prefix,nbJoints)

    print 'Test fingers.'
    for test in finger_tests:
      raw_input("Press Enter to continue...")
      moveFingers (test,prefix,nbfingers)

    print 'DONE!'

  except rospy.ROSInterruptException:
    print "program interrupted before completion"
