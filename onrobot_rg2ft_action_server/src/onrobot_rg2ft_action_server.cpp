/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a DH gripper
 * code modified from https://github.com/jr-robotics/robotiq/tree/noetic-devel/robotiq_2f_gripper_action_server
 */

#include <onrobot_rg2ft_action_server/onrobot_rg2ft_action_server.h>

namespace onrobot_rg2ft_action_server
{

OnRobotRG2FTActionServer::OnRobotRG2FTActionServer(const std::string& name, const GripperParams& params)
  : nh_()
  , as_(nh_, name, false)
  , action_name_(name)
  , gripper_params_(params)
  , is_initialized(false)
  , position_goal(0)
  , force_goal(0)
{
  as_.registerGoalCallback(boost::bind(&OnRobotRG2FTActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&OnRobotRG2FTActionServer::preemptCB, this));

  state_sub_ = nh_.subscribe(gripper_params_.state_topic_, 1, &OnRobotRG2FTActionServer::stateCB, this);
  goal_pub_ = nh_.advertise<GripperCtrl>(gripper_params_.control_topic_, 1);
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>(gripper_params_.joint_states_topic_, 1);

  as_.start();
}

void OnRobotRG2FTActionServer::goalCB()
{
  GripperCommandGoal current_goal (*(as_.acceptNewGoal()));

  if (as_.isPreemptRequested())
  {
    as_.setPreempted();
  }

  try
  {
    GripperCtrl ctrl_msg = goalToGripperCtrl(current_goal);
    goal_pub_.publish(ctrl_msg);
    position_goal = ctrl_msg.TargetWidth;
    force_goal = ctrl_msg.TargetForce;
  }
  catch (BadArgumentsError& e)
  {
    ROS_INFO("%s bad goal issued to gripper", action_name_.c_str());
  }
}

void OnRobotRG2FTActionServer::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());

  // send stop command
  // GripperCtrl ctrl_msg;
  // ctrl_msg.TargetWidth = 0;
  // ctrl_msg.TargetForce = 0;
  // ctrl_msg.Control = 0;
  // goal_pub_.publish(ctrl_msg);

  as_.setPreempted();
}

void OnRobotRG2FTActionServer::stateCB(const GripperState::ConstPtr& msg)
{
  publishJointStates(msg);

  if (!as_.isActive()) return;

  if (
    msg->ActualGripperWidth > position_goal - GOAL_TOLERANCE &&
    msg->ActualGripperWidth < position_goal + GOAL_TOLERANCE && 
    msg->GripperBusy == 0)
  {
    ROS_INFO("%s succeeded (reached target position)", action_name_.c_str());
    GripperCommandResult result;
    result.position = msg->ActualGripperWidth;
    result.effort = force_goal; // not the measured force
    result.stalled = false;
    result.reached_goal = true;
    as_.setSucceeded(result);
  }
  else if (msg->GripDetected == 1 && msg->GripperBusy == 0) {
    ROS_INFO("%s succeeded (gripped object and stalled)", action_name_.c_str());
    GripperCommandResult result;
    result.position = msg->ActualGripperWidth;
    result.effort = force_goal; // not the measured force
    result.stalled = true;
    result.reached_goal = false;
    as_.setSucceeded(result);
  }
  else
  {
    GripperCommandFeedback feedback;
    feedback.position = msg->ActualGripperWidth;
    feedback.effort = force_goal; // not the measured force
    feedback.stalled = false;
    feedback.reached_goal = false;
    as_.publishFeedback(feedback);
  }
}

GripperCtrl OnRobotRG2FTActionServer::goalToGripperCtrl(GripperCommandGoal goal) {
  double angle = goal.command.position;
  double max_effort = goal.command.max_effort;

  if (max_effort == 0) {
    max_effort = gripper_params_.default_effort_;
  }

  if (
    angle < gripper_params_.min_angle_ ||
    angle > gripper_params_.max_angle_ ||
    max_effort < gripper_params_.min_effort_ ||
    max_effort > gripper_params_.max_effort_
  ) 
  {
    throw BadArgumentsError();
  }

  double gripper_ctrl_position = mapRange(
    angle,
    gripper_params_.min_angle_,
    gripper_params_.max_angle_,
    MIN_POSITION,
    MAX_POSITION,
    true
  );

  double gripper_ctrl_effort = mapRange(
    max_effort,
    gripper_params_.min_effort_,
    gripper_params_.max_effort_,
    MIN_FORCE,
    MAX_FORCE,
    false
  );

  GripperCtrl ctrl_msg;
  ctrl_msg.TargetWidth = gripper_ctrl_position;
  ctrl_msg.TargetForce = gripper_ctrl_effort;
  ctrl_msg.Control = 1;

  return ctrl_msg;
}

void OnRobotRG2FTActionServer::publishJointStates(const GripperState::ConstPtr& gripper_state) {
  double position_radians = mapRange(
    gripper_state->ActualGripperWidth,
    MIN_POSITION,
    MAX_POSITION,
    gripper_params_.min_angle_,
    gripper_params_.max_angle_,
    true
  );

  sensor_msgs::JointState msg;
  msg.name.resize(1);
  msg.position.resize(1);

  msg.header.frame_id = "";
  msg.header.stamp = ros::Time::now();
  msg.position.at(0) = position_radians;
  msg.name.at(0) = gripper_params_.joint_name_; 

  joint_states_pub_.publish(msg);
}

double OnRobotRG2FTActionServer::mapRange(double val, double prev_min, double prev_max, double new_min, double new_max, bool reverse) {
  double ret = (val - prev_min) * (new_max - new_min) / (prev_max - prev_min) + new_min;
  if (reverse) {
    ret = new_max - ret;
  }
  return ret;
}

} // end dh_gripper_action_server namespace
