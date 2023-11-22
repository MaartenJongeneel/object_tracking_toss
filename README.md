# FSM Controller via HTC Vive UR10 Teleoperation
Teleoperation program to control the end-effector of a UR10 robot equipped with Smart Robotic's vacuum gripper, using the HTC Vive VR controller. This project makes use of the OpenVR SDK provided by Vive, and the mc_rtc control framework provided by CRNS. 

This program is originally developed to perform teleoperated robot operations to perform tosses using the UR10 robot. The program can be used to toss boxes using the UR10 robot in both a virtual environment and on a physical UR10 robot.

## Dependencies
Software programs/libraries:
- [ROS](http://wiki.ros.org/ROS/Installation)
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [mc_iam](https://gitlab.tue.nl/h2020-i-am-project/mc_iam) (_for I/O control on the robot setup in the Vanderlande innovation lab at the TU/e campus, also download and install the I/O dependency, see [here](https://gitlab.tue.nl/h2020-i-am-project/mc_iam/-/tree/feature/full_IO_control)_)
- [OpenVR SDK](https://github.com/ValveSoftware/openvr)
- [Steam VR](https://store.steampowered.com/app/250820/SteamVR/) (_requires Steam account_)

Packages for catkin workspace:
- [iam_description](https://gitlab.tue.nl/h2020-i-am-project/iam_description)
- [universal_robot package with (only) ur_description folder](https://github.com/ros-industrial/universal_robot)
- [mc_rtc_ros_control](https://github.com/mc-rtc/mc_rtc_ros_control)

Fur further details on installing these dependencies for the setup in the Innovation lab, please refer to [this guide](https://gitlab.tue.nl/h2020-i-am-project/i-am-hardware-software-integration-and-documentation/-/blob/main/UR10-mc_rtc-communication-guide.md).

Note that the catkin packages can directly be cloned in the ```src``` folder of your catkin workspace, and these packages can be build by running
```bash
cd catkin_ws
catkin_make
```

## Installation
Make sure all dependencies listed above are installed and available on your PC.

To use the UR10 robot, create a file called ur10.yaml locally (at any desired location), containing:
```yaml
path: "<Path to iam_description>/iam_description"
name: ur10_with_sr_gripper
urdf_path: "urdfs/ur10_with_sr_gripper.urdf" 
fixed: true
ref_joint_order: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
devices:
  - type: IO
    parent: ee_link

stance:
  shoulder_pan_joint: [3.14]
  shoulder_lift_joint: [-1.57]
  elbow_joint: [-1.57]
  wrist_1_joint: [-1.57]
  wrist_2_joint: [1.57]
  wrist_3_joint: [1.57]
  
minimalSelfCollisions:
    - body1: shoulder_link*
      body2: forearm_link*
      iDist: 0.2  # interaction distance: minimal distance below which the constraint becomes active
      sDist: 0.1  # safety distance: minimal allowed distance
      damping: 10
    - body1: shoulder_link*
      body2: wrist_2_link*
      iDist: 0.2
      sDist: 0.1
      damping: 10
    - body1: shoulder_link*
      body2: wrist_3_link*
      iDist: 0.2
      sDist: 0.1
      damping: 10
    - body1: shoulder_link*
      body2: gripper_link*
      iDist: 0.2
      sDist: 0.1
      damping: 10
    - body1: shoulder_link*
      body2: flange_link*
      iDist: 0.2
      sDist: 0.1
      damping: 10
    - body1: shoulder_link*
      body2: suction_cup_link*
      iDist: 0.2
      sDist: 0.1
      damping: 10
    - body1: upper_arm_link*
      body2: gripper_link*
      iDist: 0.2
      sDist: 0.1
      damping: 10
    - body1: upper_arm_link*
      body2: flange_link*
      iDist: 0.1
      sDist: 0.05
      damping: 10
    - body1: forearm_link*
      body2: flange_link*
      iDist: 0.05
      sDist: 0.01
      damping: 10
    - body1: forearm_link*
      body2: gripper_link*
      iDist: 0.1
      sDist: 0.05
      damping: 10
    - body1: forearm_link*
      body2: suction_cup_link*
      iDist: 0.2
      sDist: 0.1
      damping: 10
```
Under `minimalSelfCollisions` the distance between links that defines when a constraint must be activated, and the safety distance can be edited for each constraint, by editing `iDist` and `sDist` respectively.

Then, to use the OpenVR SDK, add the correct path to the OpenVR folder in the src/states/CMakeLists.txt file:
```
include_directories(<path to OpenVR SDK>/openvr)
```
To install the teleoperation controller, the robot including corresponding scene modules, simply build and install the cmake project:
```
mkdir build
cd build
cmake ../
make
sudo make install
```

## Running the FSM controller
The controller in this repository can be run in mc_rtc as standalone using mc-rtc-ticker. In order to do so, edit the robot and controller in the mc_rtc configuration (.config/mc_rtc/mc_rtc.yaml) to:
```yaml
MainRobot: [json_iam,"<Path to ur10.yaml>/ur10.yaml"]
Enabled: [ObjectTrackingToss]
Plugins: UR_ROS
Timestep: 0.008
```
This ensures the communcation with the UR10 is via ROS control and we use the correct timestep of the UR10 (125Hz). 
Next, open a new terminal. We start the connection with the UR10 via 
```bash
roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.104 kinematics_config:=<path_to_your>/my_robot_calibration.yaml
```

In a new terminal, run the following command to start RViZ along with a mc_rtc dedicated panel, and the robot model:
```bash
cd <object_tracking_toss>/scripts
./display.sh
```
Finally, in another terminal, run:
```bash
roslaunch mc_rtc_ros_control control.launch publish_to:=/joint_group_vel_controller/command subscribe_to:=/joint_states output_velocity:=true
```

## Teleoperation
### Teleoperation control
The teleoperation setup is controlled by VIVE's VR controller, see the figure below.

![image](https://user-images.githubusercontent.com/92452207/144387818-1c3973f5-e85a-410d-a7f9-a05f73f84b52.png)

The teleoperation system is controlled by the VR controller buttons, of which the functionality is as follows:
- Grip *(button 8)*: safety switch. Controls whether there is data transmission between the VR controller and the robot during the teleoperation state (on/off). Released button: data transmission blocked. Pressed button: data transmission resumed.
- Trackpad *(button 2)*: teleoperation control toggle. This button controls if the teleoperator is activated, when pressed: the robot's end-effector follows the VR controller's movements, and other button functionalities are activated. When released: the robot's end-effector holds its position from the moment of releasing the touchpad and controller button functionalities are deactivated.
- Trigger *(button 7)*: gripper activation. Press trigger: activates and holds vacuum while trigger is pressed. Release trigger: vacuum deactivated and blow-off activated.

During teleoperation (touchpad and grip button pressed), the VR controller controls the end-effector pose. The robot's end-effector motion is equal to the end-effector pose at the start of teleoperation (moment of pressing touchpad), plus the pose difference between the current VR controller pose and the VR controller pose at the moment of activation.

### Start teleoperation
To start the teleoperation, follow the HTC Vive's official [setup guide](https://www.vive.com/us/setup/vive-pro-hmd/). *Note: only use one VR controller, the DisplayPort cable is not required*. After succesfully connecting the VR kit, perform the room setup in SteamVR. Place the HMD somewhere safe, while visible for the basestations, *do not wear the HMD during teleoperation*. When the HMD and VR controller are correctly connected, and recognized by SteamVR the teleoperation system is ready to be used.

Before starting to use the controller, make sure you get familiar with the mc_rtc control framework using [tutorials](https://jrl-umi3218.github.io/mc_rtc/index.html) and the [API documentation](https://jrl-umi3218.github.io/mc_rtc/doxygen.html#).

Now, depending on whether you want to control the real robot to manipulate boxes, or if you want to manipulate a box in the virtual environment (recommended for first use), continue following the steps in section Real robot teleoperation or Virtual robot teleoperation.

#### Real robot teleoperation
First, follow the steps in [this guide](https://gitlab.tue.nl/h2020-i-am-project/i-am-hardware-software-integration-and-documentation/-/blob/main/UR10-mc_rtc-communication-guide.md). Then, to start using the teleoperation controller, activate the RealRobotTeleoperation state in mc_rtc. This is done under the `FSM` tab in the mc_rtc control panel, by choosing `RealRobotTeleoperation` from the `State*` list, and to click `Force transition`.

#### Virtual robot teleoperation
To start using the virtual teleoperation controller, choose `VirtualTeleoperation` from the `State*` list, and to click `Force transition`.

## mc_rtc functionality
This repository relies on several components that are made using mc_rtc.

### Robot and environment modules
The repository contains several mc_rtc modules used to build the scene in mc_rtc. For more information regarding mc_rtc scene creation, please refer to the [mc_rtc tutorial](https://jrl-umi3218.github.io/mc_rtc/tutorials/advanced/new-robot.html) or the information in [this repository](https://gitlab.tue.nl/h2020-i-am-project/software-installation-scripts-and-tutorials/-/blob/master/Environment_creation_mc_rtc.md) on the TU/e gitlab. The modules that are included in this repository are given by:

- `vi_testbench`: The table that is present in the Vanderlande innovation lab in Eindhoven.
- `conveyor_box`: A large box that acts as the conveyor on which the package is tossed.
- `toss_box1-toss_box5`: The relevant boxes for the toss scenario, as described in the scenario specification document of the I.AM.

### FSM controller
The controller is a basic FSM (finite state machine) controller in mc_rtc, which consists of several states with various functionalities:

- UR10FSMController_Initial: controls the initialization of the controller, moves robot to predetermined initial posture.
- EndeffectorTask: to manually control the robot's end-effector in mc_rtc using the computer mouse. In this state, note the `Vacuum` tab in the mc_rtc control panel where the vacuum can be switched on or off to respectively pick or release the package.
- MoveToStart: moves the UR10 robot to a conveinient configuration to start tossing operations.
- RealRobotGrabBox: controls the automatic grab of a box at a predetermined location in the Vanderlande innovation lab at the TU/e campus, using the real UR10 robot setup.
- RealRobotTeleoperation: real-world teleoperation controller, to control the real robot's end-effector using Vive's VR controller in the Vanderlande Innovation lab at the TU/e campus.
- VirtualGrabBox: controls the automatic grab of a box at any lcoation within the kinematic range of the robot in the virtual environment.
- VirtualTeleOperation: virtual teleoperation controller, to control the real robot's end-effector using Vive's VR controller. Includes the option to grab onto a box to simulate teleoperated robot manipulations.

The configuration of the FSM is defined in etc/BasicTossSceneSRGripper.in.yaml.
