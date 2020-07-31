# Safety Recovery ROS Node for Universal Robot (UR10e)
The purpose of this node is to allow for a robotic arm (a Universal Robots UR10e robot in our case) to be able to programatically recover from a protective stop. Below are some notes and observations on how the Universal Robot ROS driver works including setup and some behavior observations.

Link to the UR ROS Driver [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

## Dependencies
- `ur_dashboard_msgs`
- `ur_controllers`
- `ur_robot_driver`
- `ur_driver`
- `tf` (this may upgrade to `tf2` in the future)
- `moveit_ros_planning_interface`

## System Setup

#### Initial Startup
1. Power on the robot
2. Connect the ethernet cord to your computer and make sure that your computer's wired IP matches the one set in on the teach pendant.
3. Make sure that the robot is in remote mode and that the URcap `excontrol.urp` is loaded.
  - This can be done on the teach pendant by clicking "Load Program" and selecting the URcap file.
  - Alternatively, This can be done from the command line by first opening a TCP connection with the command `nc <robot ip> 29999` and then entering `load excontrol.urp`. The URcap should then be loaded.

#### Commands for ROS connection and control:

- `roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.100.20`
- `rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller`
  - creates a lightweight joint control gui

Many of the setup commands have been grouped together into the launch file `safety_test.launch` found in the launch folder. This will launch the driver bringup needed to connect to the robot, an rviz window, a joint controller, and the safety recovery node.

## Useful Reference Information
The robot's safety mode can be pulled from /ur_hardware_interface/safety_mode given that the bringup has been launched. It outputs a ur_dashboard_msgs/SafetyMode message.

    uint8 NORMAL=1
    uint8 REDUCED=2
    uint8 PROTECTIVE_STOP=3
    uint8 RECOVERY=4
    uint8 SAFEGUARD_STOP=5
    uint8 SYSTEM_EMERGENCY_STOP=6
    uint8 ROBOT_EMERGENCY_STOP=7
    uint8 VIOLATION=8
    uint8 FAULT=9
    uint8 VALIDATE_JOINT_ID=10
    uint8 UNDEFINED_SAFETY_MODE=11
    uint8 AUTOMATIC_MODE_SAFEGUARD_STOP=12
    uint8 SYSTEM_THREE_POSITION_ENABLING_STOP=13
    uint8 mode

Above are the different numbers and modes that the robot outputs.
The message publishing is not continuously updated, it only prints out a new status if the status has changed

The UR ROS Driver has an extensive ROS service library for handling the dashboard. More info can be found [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/ROS_INTERFACE.md#ur_robot_driver_node)

The main services that deal with safety are as follows:
- `/ur_hardware_interface/dashboard/unlock_protective_stop`
  - Dismiss a protective stop to continue robot movements. NOTE: It is the responsibility of the user to ensure the cause of the protective stop is resolved before calling this service. Unlocks the robot after encountering a protective stop
- `/ur_hardware_interface/dashboard/play`
  - Service to start UR program execution on the robot
- `/ur_hardware_interface/dashboard/stop`
  - Service to stop UR program execution on the robot

These services can be called from a node with a `ros::ServiceClient()` or through the command line with `rosservice call <service>`

## Observations and Robot Behavior
- It appears that after clearing the first protective stop, the robot will keep attempting to reach it's target, but will completely stop after a second protective stop. After this, the program trajectory must be reinitialized. This can be done in the node, however.
- Even after restarting, however, the robot likes to keep trying to reach the last point of impact. The only way to fix this is to move the robot or the object.
