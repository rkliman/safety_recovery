/**
**  ROS node to test out e-stop recovery
**  @author Randall Kliman
**/
#include <ros/ros.h>

#include <string>
#include <ur_dashboard_msgs/SafetyMode.h>
#include <ur_robot_driver/ros/dashboard_client_ros.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "safety_node");
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::NodeHandle nh;
  bool isMoving = false;

  ros::ServiceClient safety_client = nh.serviceClient<ur_dashboard_msgs::GetSafetyMode>("/ur_hardware_interface/dashboard/get_safety_mode");
  ros::ServiceClient state_client = nh.serviceClient<ur_dashboard_msgs::IsProgramRunning>("/ur_hardware_interface/dashboard/program_running");
  ros::ServiceClient unlock_client = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/unlock_protective_stop");
  ros::ServiceClient brake_client = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/brake_release");
  ros::ServiceClient play_client = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/play");
  ros::ServiceClient power_on_client = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/power_on");
  ros::ServiceClient load_client = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/load_program");
  ur_dashboard_msgs::GetSafetyMode get_mode;
  ur_dashboard_msgs::IsProgramRunning prog_state;
  ur_dashboard_msgs::Load load;
  load.request.filename = "excontrol.urp";
  std_srvs::Trigger unlock;
  std_srvs::Trigger release;
  std_srvs::Trigger play;
  std_srvs::Trigger power_on;

  //call a startup sequence
  ROS_INFO("Powering On");
  power_on_client.call(power_on);
  ROS_INFO("Unlocking");
  brake_client.call(release);
  ROS_INFO("Loading Program");
  load_client.call(load);
  ROS_INFO("Playing Program");
  play_client.call(play);


  //make sure to run ur10_e_moveit_planning_execution.launch before running this node. Otherwise "manipulator" will not be found
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  move_group.setPoseReferenceFrame("base_link");
  move_group.setNamedTarget("home");
  move_group.asyncMove();
  isMoving = true;


  while(ros::ok()) {
    // If it's stopped, restart it.
    safety_client.call(get_mode);
    state_client.call(prog_state);
    ROS_INFO("%s\t%s", get_mode.response.answer.c_str(),prog_state.response.answer.c_str());

    ///////////////////////////////////////////
    ///   Auto-restart from protective stop ///
    ///      Do not use unless certain      ///
    ///         there are no risks          ///
    ///////////////////////////////////////////

    // if (get_mode.response.answer.find("NORMAL") == std::string::npos) {
    //     ROS_INFO("Protective Stop Detected. Restarting...");
    //     ros::Duration(6).sleep();
    //     unlock_client.call(unlock);
    // }

    if (get_mode.response.answer.find("NORMAL") == std::string::npos) {
        move_group.stop();
        move_group.stop();
        isMoving = false;
        ROS_INFO("Stopping");
        ros::Duration(6).sleep();
        ROS_INFO("Unlocking");
        unlock_client.call(unlock);
        ros::Duration(3).sleep();
        play_client.call(play);
        ROS_INFO("Playing");
        // ros::Duration(3).sleep();
        do
        {
          ROS_WARN("Protective Stop detected. Please move the robot to a safe position");
          std::cout << '\n' << "Press a key to continue...";
        } while (std::cin.get() != '\n');
    }

    //Restarts program once a protective stop is cleared
    // if (get_mode.response.answer.find("NORMAL") != std::string::npos && !prog_state.response.program_running) {
    if (!prog_state.response.program_running) {
        ROS_INFO("Program Stopped, Running Program...");
        ros::Duration(2).sleep();
        play_client.call(play);
    }

    if (isMoving == false) {
      move_group.setNamedTarget("home");
      move_group.asyncMove();
      isMoving = true;
    }
    ros::Duration(0.5).sleep();
  }

  ros::waitForShutdown();
  return 0;
}
