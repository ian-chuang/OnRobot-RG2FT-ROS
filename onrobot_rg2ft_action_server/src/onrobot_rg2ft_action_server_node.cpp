#include <onrobot_rg2ft_action_server/onrobot_rg2ft_action_server.h>

int main(int argc, char** argv)
{
  // Can be renamed with standard ROS-node launch interface
  ros::init(argc, argv, "onrobot_rg2ft_action_server");
  
  // Private Note Handle for retrieving parameter arguments to the server
  ros::NodeHandle private_nh("~");

  std::string action_server_name;
  private_nh.param<std::string>("action_server_name", action_server_name, "gripper_controller/gripper_cmd");

  // Fill out DH-Gripper Params
  onrobot_rg2ft_action_server::GripperParams cparams;
  
  private_nh.param<double>("min_angle", cparams.min_angle_, 0.0);
  private_nh.param<double>("max_angle", cparams.max_angle_, 0.93);
  private_nh.param<double>("min_effort", cparams.min_effort_, 3);
  private_nh.param<double>("max_effort", cparams.max_effort_, 40);
  private_nh.param<double>("default_effort", cparams.default_effort_, 10);
  private_nh.param<std::string>("control_topic", cparams.control_topic_, "gripper/ctrl");
  private_nh.param<std::string>("state_topic", cparams.state_topic_, "gripper/states");
  private_nh.param<std::string>("joint_states_topic", cparams.joint_states_topic_, "joint_states");
  private_nh.param<std::string>("joint_name", cparams.joint_name_, "left_outer_knuckle_joint");

  ROS_INFO("Initializing OnRobot RG2-FT Gripper action server: %s", action_server_name.c_str());

  // The name of the gripper -> this server communicates over name/inputs and name/outputs
  onrobot_rg2ft_action_server::OnRobotRG2FTActionServer gripper (action_server_name, cparams);

  ROS_INFO("action-server spinning for OnRobot RG2-FT Gripper: %s", action_server_name.c_str());
  ros::spin();
}
