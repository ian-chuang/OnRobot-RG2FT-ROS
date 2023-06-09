/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a DH Gripper
 * code modified from https://github.com/jr-robotics/robotiq/tree/noetic-devel/robotiq_2f_gripper_action_server
 */

#ifndef ONROBOT_RG2FT_ACTION_SERVER_H
#define ONROBOT_RG2FT_ACTION_SERVER_H

// STL
#include <string>
// ROS standard
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <sensor_msgs/JointState.h>
// Repo specific includes
#include <onrobot_rg2ft_msgs/RG2FTCommand.h>
#include <onrobot_rg2ft_msgs/RG2FTState.h>

namespace onrobot_rg2ft_action_server
{

// DH gripper control parameters
#define MIN_POSITION 0
#define MAX_POSITION 1000
#define MIN_FORCE 30
#define MAX_FORCE 400
#define GOAL_TOLERANCE 20

typedef onrobot_rg2ft_msgs::RG2FTCommand GripperCtrl;
typedef onrobot_rg2ft_msgs::RG2FTState GripperState;

typedef control_msgs::GripperCommandGoal GripperCommandGoal;
typedef control_msgs::GripperCommandFeedback GripperCommandFeedback;
typedef control_msgs::GripperCommandResult GripperCommandResult;

struct BadArgumentsError {};

/**
 * @brief Structure containing the parameters necessary to translate
 *        GripperCommand actions to register-based commands to a
 *        particular gripper (and vice versa).
 *
 *        The min gap can be less than zero. This represents the case where the 
 *        gripper fingers close and then push forward.
 */
struct GripperParams
{
  double min_angle_; // radians
  double max_angle_;
  double min_effort_; // N / (Nm) ???
  double max_effort_;
  double default_effort_;
  std::string control_topic_;
  std::string state_topic_;
  std::string joint_states_topic_;
  std::string joint_name_;
};

/**
 * @brief The DHGripperActionServer class. Takes as arguments the name of the gripper it is to command,
 *        and a set of parameters that define the physical characteristics of the particular gripper.
 *        
 *        Listens for messages on input and publishes on output. Remap these.
 */
class OnRobotRG2FTActionServer
{
public:
  OnRobotRG2FTActionServer(const std::string& name, const GripperParams& params);

  // These functions are meant to be called by simple action server
  void goalCB();
  void preemptCB();
  void stateCB(const GripperState::ConstPtr& msg);

private:
  GripperCtrl goalToGripperCtrl(GripperCommandGoal goal);
  void issueInitialization();
  void publishJointStates(const GripperState::ConstPtr& gripper_state);
  double mapRange(double val, double prev_min, double prev_max, double new_min, double new_max, bool reverse);

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;

  ros::Subscriber state_sub_; // Subs to grippers "input" topic
  ros::Publisher goal_pub_; // Pubs to grippers "output" topic
  ros::Publisher joint_states_pub_;

  std::atomic<int> position_goal;
  std::atomic<int> force_goal;
  std::atomic<bool> is_initialized;

  /* Used to translate GripperCommands in engineering units
   * to/from register states understood by gripper itself. Different
   * for different models/generations of DH grippers */
  GripperParams gripper_params_;

  std::string action_name_;
};

}
#endif
