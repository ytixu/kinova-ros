# JACO ON TABLE

## Milestone 1: Simulator
-------------------------

Added table + Kinect sensor in the simulation:

- Code for the URDF and meshes were taking from https://github.com/JenniferBuehler/jaco-arm-pkgs/tree/master/jaco_tutorial/jaco_on_table
- Dependency for adding Kinect sensor: https://github.com/JenniferBuehler/common-sensors

### To Run Gazebo and Moveit

```
roslaunch kinova_gazebo robot_table.launch
```

The simulation is paused at launch time. Unpause it, wait until the arm pauses. Then you can launch Moveit.

```
roslaunch j2n6s300_moveit_config j2n6s300_gazebo_demo.launch
```

Then the robot may not be in its home position. You can run

```
rosrun kinova_control move_robot.py j2n6s300
```
or use the Moveit to set the goal to the home position.

### For cartesian planning

You can use https://github.com/ros-industrial-consortium/fermi


### Adding Xtion

Xtion sensor can be added to the simulation by uncommenting the section at the send of `kinova_description/urdf/arm_to_table.xacro` file.


## Milestone 2: Camera Calibration
----------------------------------

### Requirements

#### ar_tools
http://wiki.ros.org/ar_tools

I had to install vision_opencv (http://wiki.ros.org/vision_opencv) under the same workspace otherwise it wouldn't build correctly (as in this [discussion](http://answers.ros.org/question/209293/error-trying-to-catkin_make-ar_pose-package/)).

You can run some tests after installing.

```
cd ar_tools/ar_pose/demo
./setup_demos
```

This will download the data files from their ar_data repo. Tests are:

```
roslaunch ar_pose demo_single.launch
roslaunch ar_pose demo_multi.launch
roslaunch ar_pose demo_reverse.launch

```

#### uvc_camera
http://wiki.ros.org/uvc_camera

Camera driver for ar_tools.

#### image_proc
http://wiki.ros.org/image_proc

Dependency for ar_tools.

#### camera_calibration
http://wiki.ros.org/camera_calibration

For internal calibration of the camera. It has name conflict with image_proc, so we can use it independently from the workspace with ar_tools to get up the camera info.
---

See original readme file from the Kinova repo in `README_original.md`.