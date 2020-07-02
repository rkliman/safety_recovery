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

  //ur_driver::DashboardClientROS dashboard_client(nh, "192.168.100.10");
  //std_srvs::Trigger dash_client;
  ros::ServiceClient safety_client = nh.serviceClient<ur_dashboard_msgs::GetSafetyMode>("/ur_hardware_interface/dashboard/get_safety_mode");
  ros::ServiceClient state_client = nh.serviceClient<ur_dashboard_msgs::IsProgramRunning>("/ur_hardware_interface/dashboard/program_running");
  ros::ServiceClient unlock_client = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/unlock_protective_stop");
  ros::ServiceClient play_client = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/play");
  ros::ServiceClient power_on_client = nh.serviceClient<std_srvs::Trigger>("/ur_hardware_interface/dashboard/power_on");
  ur_dashboard_msgs::GetSafetyMode get_mode;
  ur_dashboard_msgs::IsProgramRunning prog_state;
  std_srvs::Trigger unlock;
  std_srvs::Trigger play;
  std_srvs::Trigger power_on;

  power_on_client.call(power_on);
  play_client.call(play);


  //make sure to run ur10_e_moveit_planning_execution.launch before running this node. Otherwise "manipulator" will not be found
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  move_group.setPoseReferenceFrame("base_link");
  // geometry_msgs::Point target_p;
  // target_p.x = 1;
  // target_p.y = 0;
  // target_p.z = 0;
  // geometry_msgs::Quaternion target_o;
  // target_o.x = 0;
  // target_o.y = 0;
  // target_o.z = 0;
  // target_o.w = 0;
  // geometry_msgs::Pose move_target;
  // move_target.position = target_p;
  // move_target.orientation = target_o;
  // move_group.setPoseTarget(move_target);
  move_group.setNamedTarget("home");
  move_group.asyncMove();


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
        ros::Duration(3).sleep();
    }

    //Restarts program once a protective stop is cleared
    // if (get_mode.response.answer.find("NORMAL") != std::string::npos && !prog_state.response.program_running) {
    if (!prog_state.response.program_running) {
        ROS_INFO("Program Stopped, Running Program...");
        ros::Duration(2).sleep();
        play_client.call(play);
        move_group.setNamedTarget("home");
        move_group.asyncMove();
        //goToPose();
    }
    ros::Duration(0.5).sleep();
  }

  ros::waitForShutdown();
  return 0;
}
