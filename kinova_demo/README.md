### TESTS

1) Connect with USB and test SDK
2) Test ROS package

	roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300

	rosrun rviz rviz

		i) Select root as the world frame

		ii) Add InteractiveMarkers, click on the right of Updated Topic of added interactive marker, and select the topic

		/j2s6s300_interactive_control_Joint/update
			for joint control

		/j2s6s300_interactive_control_Cart/update
			for cartesian control

	Test services

	rosservice call /j2s6s300_driver/in/home_arm
	rosservice call /j2s6s300_driver/in/start
	rosservice call /j2s6s300_driver/in/stop


3) Test file

	rosrun kinova_demo testActionSvr.py j2s6s300

4) Useful topics

	i) Joint position

		/j2s6s300_driver/out/joint_state
			Joint names, velocity and effort information. (However, the effort is a place holder for further verstion.)

		/j2s6s300_driver/out/joint_angles
			In degree

	 	/j2s6s300_driver/out/state/position
	 		In radians including finger information

	ii) Cartesian position

		/j2s6s300_driver/out/tool_pose

		/j2s6s300_driver/out/tool_wrench
			Wrench of end-effector

		(can try service AddPoseToCartesianTrajectories in rviz)

	iii) Finger position

		/j2s6s300_driver/out/finger_position
			0 (fully-open) to 6400 (fully-close)

5) Test joint, cartesian, finger, velocity and torque controls

	rosrun kinova_demo joints_action_client.py -v -r j2s6s300 degree -- 180 180 180 180 180 180

	rosrun kinova_demo pose_action_client.py -v -r j2s6s300 mdeg --

	rosrun kinova_demo fingers_action_client.py j2s6s300 percent -- 50 50 50

	rostopic pub -r 100 /j2n6s300_driver/in/joint_torque kinova_msgs/JointTorque "{joint1: 0.0, joint2: 0.0, joint3: 0.0, joint4: 0.0, joint5: 0.0, joint6: 1.0}"

	...

6) Test Cartesian Admittance mode (User can control the robot by manually guiding it by hand)

	rosservice call /j2n6s300_driver/in/start_force_control
	rosservice call /j2n6s300_driver/in/stop_force_control