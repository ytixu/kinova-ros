# Installation

You can follow the installation instruction in the original package from Kinova: https://github.com/Kinovarobotics/kinova-ros#installation (i.e.: just clone this repo under `<your_catkin_workspace>/src` and run `catkin_make`).

You can also download the SDK from http://www.kinovarobotics.com/innovation-robotics/products/software/ which can serve as a primary test for checking the arm's connection with your device. 

# JACO ON TABLE

## Milestone 1: Simulator

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

### Testing the arm

You can find the test file as the original Kinova repo in `kinova_demo/nodes/kinova_demo/textActionSvr.py`.

I wrote a simple test file for the simulator here `kinova_control/src/move_robot_test.py`. Torque and velocity control are apparently not implemented for the simulator.


## Milestone 2: Camera Calibration

### ARToolKit

ar_tools http://wiki.ros.org/ar_tools

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

Note: I had to install [vision_opencv](http://wiki.ros.org/vision_opencv) under the same workspace otherwise it wouldn't build correctly (as in this [discussion](http://answers.ros.org/question/209293/error-trying-to-catkin_make-ar_pose-package/)).

If may need to switch between camera devices in the launch files by changing `/dev/video1` to `/dev/video0` and vice-versa if you have a laptop camera and an external usb camera.

#### ~~uvc_camera~~
~~http://wiki.ros.org/uvc_camera Camera driver for ar_tools.~~

~~Note: the node for the current version is 'uvc_camera_node' instead of 'camera_node', which you may need to change in the ar_pose launch files.~~

#### image_proc
http://wiki.ros.org/image_proc Image rectification for ar_tools.

#### camera_calibration
http://wiki.ros.org/camera_calibration

For intrinsic calibration of the camera. It has name conflict with image_proc, so we can use it independently from the workspace with ar_tools to set up the camera info.

Note: The checkerboard need to be on a rigid surface. I was a bit confused with their GUI because the COMMIT button doesn't work, so I went converting the INI formatted output to YAML using [camera_calibration_parsers](http://wiki.ros.org/camera_calibration_parsers). Save the .yaml file as `camera_calibration.yaml` under the folder `src/uvc_camera/uvc_camera`. (This is the name specified in ar_pose launch files.)

#### Testing

With a monocular camera, after calibration, unzip the markers in the 'ar_pose/data/4x4' and pick the ones specified in the launch files.

```
roslaunch ar_pose ar_pose_single.launch
roslaunch ar_pose ar_pose_multi.launch
roslaunch ar_pose ar_pose_reverse.launch

```

### Adding ARToolKit Marker in Gazebo

Using [gazebo_models](https://github.com/mikaelarguedas/gazebo_models), we can create mesh for .png markers. Marker link is added to `kinova_description/urdf/arm_to_table.xacro`.

### Hand to Eye Calibration

vision_visp/visp_hand2eye_calibration http://wiki.ros.org/visp_hand2eye_calibration?distro=indigo

#### Extrinsic calibration routine

Tape the AR marker onto the robot's hand. Run

```
roslaunch kinova_camera extrinsic_calibration.launch
rosrun kinova_camera calibrator
```

In the launch file, you can specify the size and the image source location of the marker.


## Milestone 3: Camera Calibration with Kinect

Launch Kinect with [freenect_launch](http://wiki.ros.org/freenect_launch), which will replace `uvc_camera`.

Calibrate as in this [tutorial](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration). For convinience, I saved the intrinsics in `kinova_camera/config/intrinsic_calibration.yaml`, which is specified in `kinova_camera/launch/extrinsic_calibration.launch`

Remap the message in `ar_tools/ar_pose/include/ar_single.h` (and recompile).
```
const std::string cameraImageTopic_ = "/camera/rgb/image_raw";
const std::string cameraInfoTopic_  = "/camera/rgb/camera_info";
```

#### Testing

To test if camera works with `ar_pose`, run

```
roslaunch kinova_camera extrinsic_calibration.launch
rostopic echo /ar_pose_marker
```

The default marker is `4x4_98.patt, with size 35`.

THIS SECTION IS NOT FINALLISED 

---

See original readme file from the Kinova repo in `README_original.md`.
