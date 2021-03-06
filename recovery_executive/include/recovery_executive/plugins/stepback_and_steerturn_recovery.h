/**
 *
 * @file stepback_and_steerturn_recovery.h
 * @brief stepback and steerturn recovery class
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef RECOVERY_EXECUTIVE_STEPBACK_AND_STEERTURN_RECOVERY_H
#define RECOVERY_EXECUTIVE_STEPBACK_AND_STEERTURN_RECOVERY_H

#include <controller_executive/trajectory_planner/costmap_model.h>
#include <costmap_2d/costmap_helper.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <mw_core/utility_functions.h>
#include <recovery_executive/base_recovery.h>

namespace recovery
{

/// Recovery behavior that takes a given twist and tries to execute it for up to
/// d seconds, or until reaching an obstacle.
class StepBackAndSteerTurnRecovery : public recovery::AbstractRecovery
{
public:

  /// Doesn't do anything: initialize is where the actual work happens
  StepBackAndSteerTurnRecovery();

  ~StepBackAndSteerTurnRecovery() override;

  /// Initialize the parameters of the behavior
  void initialize(const std::string &name, const TFPtr &tf_listener_ptr) override;

  uint32_t runBehavior(std::string& message) override;

  bool cancel() override;

private:
  enum COSTMAP_SEARCH_MODE
  {
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    BACKWARD
  };

  enum TURN_DIRECTION
  {
    LEFT,
    RIGHT,
  };

  enum TURN_NO
  {
    FIRST_TURN = 0,
    SECOND_TURN = 1,
  };
  static const int CNT_TURN = 2;

  geometry_msgs::Twist TWIST_STOP;

  geometry_msgs::Pose2D getCurrentLocalPose() const;
  geometry_msgs::Twist scaleGivenAccelerationLimits(const geometry_msgs::Twist& twist, double time_remaining) const;
  geometry_msgs::Pose2D getPoseToObstacle(const geometry_msgs::Pose2D& current, const geometry_msgs::Twist& twist) const;
  double normalizedPoseCost(const geometry_msgs::Pose2D& pose) const;
  geometry_msgs::Twist transformTwist(geometry_msgs::Pose2D& pose) const;
  void moveSpacifiedLength (geometry_msgs::Twist twist, double length, COSTMAP_SEARCH_MODE mode = FORWARD) const;
  double getCurrentDiff(geometry_msgs::Pose2D initialPose, COSTMAP_SEARCH_MODE mode = FORWARD) const;
  double getCurrentDistDiff(geometry_msgs::Pose2D initialPose, double distination, COSTMAP_SEARCH_MODE mode = FORWARD) const;
  double getMinimalDistanceToObstacle(COSTMAP_SEARCH_MODE mode) const;
  int determineTurnDirection();
  double getDistBetweenTwoPoints(geometry_msgs::Pose2D pose1, geometry_msgs::Pose2D pose2) const;


  ros::NodeHandle nh_;
  costmap_2d::CostmapHelper::Ptr global_costmap_;
  costmap_2d::CostmapHelper::Ptr local_costmap_;

  std::string robot_base_frame_;
  std::string global_frame_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher recover_run_pub_;
  bool initialized_;

  // Memory owned by this object
  // Mutable because footprintCost is not declared const
  mutable trajectory_planner::CostmapModel* world_model_;

  geometry_msgs::Twist base_frame_twist_;

  double duration_;
  double linear_speed_limit_;
  double angular_speed_limit_;
  double linear_acceleration_limit_;
  double angular_acceleration_limit_;
  double controller_frequency_;
  double simulation_frequency_;
  double simulation_inc_;

  bool only_single_steering_;
  int trial_times_;
  double obstacle_patience_;
  double obstacle_check_frequency_;
  double sim_angle_resolution_;
  //-- back
  double linear_vel_back_;
  double step_back_length_;
  double step_back_timeout_;
  //-- steer
  double linear_vel_steer_;
  double angular_speed_steer_;
  double turn_angle_;
  double steering_timeout_;
  //-- forward
  double linear_vel_forward_;
  double step_forward_length_;
  double step_forward_timeout_;

  bool cancel_;
};

} // namespace recovery

#endif  // RECOVERY_EXECUTIVE_STEPBACK_AND_STEERTURN_RECOVERY_H
