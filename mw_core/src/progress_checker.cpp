/**
 *
 * @file progress_checker.cpp
 * @brief checks if robot has moved enough
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#include "mw_core/progress_checker.h"
#include <cmath>

namespace mw_core {

ProgressChecker::ProgressChecker(double radius_tolerance, double angular_tolerance)
{
  radius_ = radius_tolerance;
  angular_ = angular_tolerance;
}

double ProgressChecker::check(geometry_msgs::PoseStamped & current_pose)
{
  if ((!baseline_pose_set_) || (is_robot_moved_enough(current_pose.pose))) {
    reset_baseline_pose(current_pose.pose);
    return -1;
  }
  return (ros::Time::now() - baseline_time_).toSec();
}

void ProgressChecker::reset_baseline_pose(const geometry_msgs::Pose & pose)
{
  baseline_pose_ = pose;
  baseline_time_ = ros::Time::now();
  baseline_pose_set_ = true;
}

bool ProgressChecker::is_robot_moved_enough(const geometry_msgs::Pose & pose)
{
  return mw_core::distance(pose, baseline_pose_) >= radius_ || mw_core::angle(pose, baseline_pose_) >= angular_;
}

}  // namespace mw_core
