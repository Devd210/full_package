/**
 *
 * @file pure_pursuit_controller.h
 * @brief The pure pursuit controller class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef CONTROLLER_EXECUTIVE_PURE_PURSUIT_CONTROLLER_H
#define CONTROLLER_EXECUTIVE_PURE_PURSUIT_CONTROLLER_H

#include <vector>
#include <string>
#include <cmath>
#include <exception>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <angles/angles.h>

#include <mw_core/utility_functions.h>
#include <mw_core/data_types/geometry_types.h>

#include <controller_executive/PurePursuitControllerConfig.h>
#include <dynamic_reconfigure/server.h>

#include "controller_executive/base_controller.h"

#define MAX_VEL_DELTA  0.2
#define MAX_OMEGA_DELTA  0.2

namespace controller {

class PurePursuitControllerException : public std::exception {

 private:
  int error_;

 public:

  explicit PurePursuitControllerException(int error) : error_(error) {}

  int getErrorCode() {
    return error_;
  }

  const char *what() const noexcept override {
    switch (error_) {
      case 0: return "All is well.";
      case 1: return "Theta deviation out of bound.";
      case 2: return "Path deviation out of bound.";
      case 3: return "Error undefined";
      case 4: return "Not getting ego_pose.";
      case 5: return "Start_sanity false. Check Robot Location.";
      case 6: return "Online radius has gone below the min radius of turn.";
      case 7: return "Controller is throwing out velocities more than max_vel_";
      case 8: return "Controller is throwing out omega more than max_omega_";
      case 9: return "There is a sanity issue in the compute function something funny is happerning.";
      case 10: return "There is problem transforming point in robot frame ";
      case 11: return "max_accel set to 0. Divide by 0 error";
      case 12: return "max_y_deviation set to 0. Divide by 0 error";
      case 13: return "theta_saturation_lim_max set to 0. Divide by 0 error";
      case 14: return "min_radius and max_radius are equal. Divide by 0 error";
      case 15: return "The input slope of two points are equal, at getIntersectio(). Divide by 0 error";
      default: return "Unknown ERROR CODE";
    }
  }
};

class PurePursuitControllerParams {

 public:
  PurePursuitControllerParams() :
      min_lookahead(0.0),
      max_lookahead(0.0),
      closest_point_index_search(0.0),
      max_acceleration(0.0),
      max_vel(0.0),
      min_vel(0.0),
      max_omega(0.0),
      max_radius(0.0),
      min_radius(0.0),
      max_path_dev(0.0),
      max_theta_dev(0.0),
      transform_tolerance(0.5),
      max_omega_radius(0.0),
      max_y_deviation(0.0),
      theta_saturation_lim_max(0.0),
      robot_base_frame("base_link"),
      map_frame("map"),
      tf_timeout(0.0) {}

  double min_lookahead;
  double max_lookahead;

  double closest_point_index_search;

  double max_acceleration;

  double max_vel;
  double min_vel;

  double max_omega;

  double max_radius;
  double min_radius;

  double max_path_dev;
  double max_theta_dev;

  double transform_tolerance;

  double max_omega_radius;
  double max_y_deviation;

  double theta_saturation_lim_max;

  std::string robot_base_frame;
  std::string map_frame;

  double tf_timeout;
};

class PurePursuitController : public AbstractController {

 public:

  PurePursuitController();

  ~PurePursuitController() override;

  // AbstractController functions
  unsigned int computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                       const geometry_msgs::TwistStamped &velocity,
                                       geometry_msgs::TwistStamped &cmd_vel,
                                       std::string &message) override;

  bool isGoalReached(double dist_tolerance, double angle_tolerance) override;

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

  bool cancel() override;

  void reset() override;

  // Resets velocity to zero
  void resetCmdVel();

  geometry_msgs::TwistStamped getCmdVel() {
    return cmd_vel_;
  }

  double getPathDeviation() {
    return path_deviation_;
  }

  double getDistanceToGoal() {
    return distance_to_goal_;
  }

  void reconfigureCB(controller_executive::PurePursuitControllerConfig &config, uint32_t level);

 private:

  // computes everything remaining, and calls all reminaing functions
  bool compute();

  // ------------------- all the computation related functions ----------------------------
  // calculates the euclidean distance
  double calculateXYDeviation(const mw_core::Point &pose1, const mw_core::Point &pose2);

  // difference in heading pose1 - pose2
  double calculateThetaDeviation(const mw_core::Point &pose1, const mw_core::Point &pose2);

  // find the pose from the plan_ closes to the current ego pose
  int findClosestPose();

  // calculates the radius of curvature from the closes point to the point next to it
  // internally calls get Curvature
  double getRadiusOfCurvatureOfGoalPoints();

  // finds the radius of curvature between two points having heading
  double getRadiusOfCurvature(mw_core::Point pose1, mw_core::Point pose2);

  // figures out the lookahead value based on curvatur and interpolation
  // puru: is the formaula correct? where is it taken from? any references?
  // puru: what does it exactly return, an index to a pose or somethig else? since it an int
  double getLookahead(const double &curvature);

  // calculates target velocity based on interpolation of target radius and ramping
  double calculateVelocity(double _target_radius);

  mw_core::Point getIntersection(bool is_vert_1, double m_1, double c_1, bool is_vert_2, double m_2, double c_2);
  
  // gets the target pose from the plan_
  // puru: it treats lookahead as difference in index of plan_, is it what you want?
  mw_core::Point getTargetPose(const double &lookahead);

  // convert the pose (in) in the robot/base frame, and writes it in 'out'
  bool getPointRobot(geometry_msgs::PointStamped &in, geometry_msgs::PointStamped &out);

  // get the radius of the turn needed to make in order to reach the target
  double getTargetRadius(mw_core::Point p1, mw_core::Point p2);

  // gives the difference between the current heading and the heading needed to reach the target
  double headingError(mw_core::Point current_pose, mw_core::Point target_pose);

  double improveLookahead(double lookahead);

  // a proportional controller over the theta
  double calculateOmegaAlternate(double theta_err);

  // converts a PoseStamped array to Pose2D array
  std::vector<mw_core::Point> convertToPose2D(std::vector<geometry_msgs::PoseStamped> plan);

  // calculates euler yaw angle from quaternion
  double getYawFromPose(const geometry_msgs::PoseStamped pose) {
    tf2::Quaternion q(pose.pose.orientation.x,
                      pose.pose.orientation.y,
                      pose.pose.orientation.z,
                      pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  double principleTheta(double theta) {
    while (theta <= -M_PI) {
      theta += 2*M_PI;
    }
    while (theta > M_PI) {
      theta -= 2*M_PI;
    }
    return theta;
  }

 private:

  mw_core::Point current_pose_;
  geometry_msgs::TwistStamped current_velocity_;

  std::vector<mw_core::Point> plan_;

  // for executing the call back queue. This arrangement is due diagnostics
  int error_code_;

  bool is_goal_reached_;
  double distance_to_goal_;

  double distance_from_start_;

  // starting and end pose of the plan
  mw_core::Point start_pose_;
  mw_core::Point goal_pose_;

  double path_deviation_;

  geometry_msgs::TwistStamped cmd_vel_;

  bool omega_positive_dir_;

  int closest_pose_idx_;

  PurePursuitControllerParams params_;

  double ramp_distance_;

  bool holonomic_robot_;

  bool on_the_spot_turn_allowed_;

  std::string drive_type_;

  dynamic_reconfigure::Server<controller_executive::PurePursuitControllerConfig> *dsrv_;
  controller_executive::PurePursuitControllerConfig default_config_;

  const double LOOKAHEAD_POINT_DISTANCE = 0.01;

  ros::Publisher pubLookaheadGoal_;
};

}

#endif //CONTROLLER_EXECUTIVE_PURE_PURSUIT_CONTROLLER_H
