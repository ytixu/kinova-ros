#! /usr/bin/env python
""" Test trajectories """

import rospy

if __name__ == '__main__':
  try:
    rospy.init_node('move_robot_using_trajectory_msg_test')

    # go to home position first
    moveJoint ([0.0,2.9,1.3,4.2,1.4,0.0],prefix,nbJoints)
    moveFingers ([1,1,1],prefix,nbfingers)

    # TEST HERE

  except rospy.ROSInterruptException:
    print "program interrupted before completion"
