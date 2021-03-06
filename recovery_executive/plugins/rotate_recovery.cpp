/**
 *
 * @file rotate_recovery.cpp
 * @brief rotate recovery class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>

#include <mw_core/utility_functions.h>

#include <recovery_executive/plugins/rotate_recovery.h>

// register this planner as a Recovery plugin
PLUGINLIB_EXPORT_CLASS(recovery::RotateRecovery, recovery::AbstractRecovery)

namespace recovery {

RotateRecovery::RotateRecovery()
: initialized_(false),
  cancel_(false)
{
}

void RotateRecovery::initialize(const std::string &name, const TFPtr &tf_listener_ptr) {
  if (!initialized_)
  {
    name_ = name;
    tf_listener_ptr_ = tf_listener_ptr;

    ros::NodeHandle nh;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/trajectory_planner");

    private_nh.param("frequency", frequency_, 20.0);
    private_nh.param("angle", angle_, M_PI);
    private_nh.param("global_frame", global_frame_, std::string("map"));
    private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    blp_nh.param("acc_lim_theta", acc_lim_th_, 3.2);
    blp_nh.param("max_vel_theta", max_rotational_vel_, 1.0);
    blp_nh.param("min_in_place_vel_theta", min_rotational_vel_, 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    initialized_ = true;
    cancel_ = false;
  }
  else
  {
    ROS_ERROR("[Rotate Recovery]: You should not call initialize twice on this object, doing nothing");
  }
}

RotateRecovery::~RotateRecovery()
{
}

uint32_t RotateRecovery::runBehavior(std::string &message) {
  if (!initialized_)
  {
    ROS_ERROR("[Rotate Recovery]: This object must be initialized before runBehavior is called");
    return 1;
  }

  cancel_ = false;

  ROS_WARN("[Rotate Recovery]: Rotate recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  geometry_msgs::PoseStamped global_pose;
  if (!mw_core::getRobotPose(*tf_listener_ptr_, robot_base_frame_,
    global_frame_, ros::Duration(1.0), global_pose)) {
    ROS_ERROR("[Rotate Recovery]: Could not get robot pose");
    return 1;
  }

  double current_angle = tf2::getYaw(global_pose.pose.orientation);
  double start_angle = current_angle;
  double angle_traveled;

  geometry_msgs::Twist cmd_vel;

  while (n.ok() && !cancel_)
  {
    // Update Current Angle
    if (!mw_core::getRobotPose(*tf_listener_ptr_, robot_base_frame_,
                               global_frame_, ros::Duration(1.0), global_pose)) {
      ROS_ERROR("[Rotate Recovery]: Could not get robot pose");
      return 1;
    }

    current_angle = tf2::getYaw(global_pose.pose.orientation);

    // compute the distance left to rotate
    angle_traveled = std::fabs(angles::shortest_angular_distance(current_angle, start_angle));
    double dist_left = std::fabs(angle_ - angle_traveled);

    if (dist_left < tolerance_)
    {
      break;
    }

    // compute the velocity that will let us stop by the time we reach the goal
    double vel = sqrt(2 * acc_lim_th_ * dist_left);

    // make sure that this velocity falls within the specified limits
    vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = vel;

    vel_pub.publish(cmd_vel);

    r.sleep();
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  vel_pub.publish(cmd_vel);

  return 0;
}

bool RotateRecovery::cancel() {
  cancel_ = true;
  return true;
}

};  // namespace rotate_recovery
