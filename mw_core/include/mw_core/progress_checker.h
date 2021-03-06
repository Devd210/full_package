/**
 *
 * @file progress_checker.h
 * @brief checks if robot has moved enough
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef MW_CORE_PROGRESS_CHECKER_H
#define MW_CORE_PROGRESS_CHECKER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <mw_core/utility_functions.h>
#include <tf2/utils.h>

namespace mw_core
{
class ProgressChecker
{
public:
  explicit ProgressChecker(double radius_tolerance, double angular_tolerance);

  double check(geometry_msgs::PoseStamped & current_pose);

  void reset() {baseline_pose_set_ = false;}

protected:

  bool is_robot_moved_enough(const geometry_msgs::Pose & pose);

  void reset_baseline_pose(const geometry_msgs::Pose & pose);

  double radius_;
  double angular_;

  geometry_msgs::Pose baseline_pose_;
  ros::Time baseline_time_;

  bool baseline_pose_set_{false};
};
}  // namespace mw_core

#endif //MW_CORE_PROGRESS_CHECKER_H
