/**
 *
 * @file back_up_recovery.cpp
 * @brief back up recovery class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>

#include <mw_core/utility_functions.h>

#include <recovery_executive/plugins/back_up_recovery.h>

// register this planner as a Recovery plugin
PLUGINLIB_EXPORT_CLASS(recovery::BackUpRecovery, recovery::AbstractRecovery)

namespace recovery {

BackUpRecovery::BackUpRecovery()
  : initialized_(false),
    cancel_(false)
{
}

void BackUpRecovery::initialize(const std::string &name, const TFPtr &tf_listener_ptr) {
  if (!initialized_)
  {
    name_ = name;
    tf_listener_ptr_ = tf_listener_ptr;

    ros::NodeHandle nh;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/trajectory_planner");

    private_nh.param("frequency", frequency_, 20.0);
    private_nh.param("distance", distance_, 0.5);
    private_nh.param("global_frame", global_frame_, std::string("map"));
    private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    blp_nh.param("acc_lim_x", acc_lim_x_, 0.5);
    blp_nh.param("min_vel_x", min_vel_x_, -0.25);
    blp_nh.param("max_vel_x", max_vel_x_, 0.25);
    blp_nh.param("xy_goal_tolerance", tolerance_, 0.1);

    initialized_ = true;
    cancel_ = false;
  }
  else
  {
    ROS_ERROR("[Back Up Recovery]: You should not call initialize twice on this object, doing nothing");
  }
}

BackUpRecovery::~BackUpRecovery()
{
}

uint32_t BackUpRecovery::runBehavior(std::string &message) {
  if (!initialized_)
  {
    ROS_ERROR("[Back Up Recovery]: This object must be initialized before runBehavior is called");
    return 1;
  }

  cancel_ = false;

  ROS_WARN("[Back Up Recovery]: Back up recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  geometry_msgs::PoseStamped initial_pose;
  geometry_msgs::PoseStamped current_pose;

  if (!mw_core::getRobotPose(*tf_listener_ptr_, robot_base_frame_,
                             global_frame_, ros::Duration(1.0), initial_pose)) {
    ROS_ERROR("[Back Up Recovery]: Could not get robot pose");
    return 1;
  }

  current_pose = initial_pose;

  double diff_x;
  double diff_y;
  double distance;

  geometry_msgs::Twist cmd_vel;

  while (n.ok() && !cancel_)
  {
    // Update Current Angle
    if (!mw_core::getRobotPose(*tf_listener_ptr_, robot_base_frame_,
                               global_frame_, ros::Duration(1.0), current_pose)) {
      ROS_ERROR("[Back Up Recovery]: Could not get robot pose");
      return 1;
    }

    diff_x = initial_pose.pose.position.x - current_pose.pose.position.x;
    diff_y = initial_pose.pose.position.y - current_pose.pose.position.y;
    distance = sqrt(diff_x * diff_x + diff_y * diff_y);

    double dist_left = std::fabs(std::fabs(distance_) - distance);

    if (dist_left < tolerance_)
    {
      break;
    }

    // compute the velocity that will let us stop by the time we reach the goal
    double vel = sqrt(2 * acc_lim_x_ * dist_left);

    // make sure that this velocity falls within the specified limits
    vel = std::min(std::max(vel, min_vel_x_), max_vel_x_);

    cmd_vel.linear.x = distance_ < 0 ? -vel : vel;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;

    vel_pub.publish(cmd_vel);

    r.sleep();
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  vel_pub.publish(cmd_vel);

  return 0;
}

bool BackUpRecovery::cancel() {
  cancel_ = true;
  return true;
}

};  // namespace recovery
