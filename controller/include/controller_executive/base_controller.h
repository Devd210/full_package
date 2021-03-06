/**
 *
 * @file base_controller.h
 * @brief The base controller class. Every controller must inherit this class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef CONTROLLER_EXECUTIVE_BASE_CONTROLLER_H
#define CONTROLLER_EXECUTIVE_BASE_CONTROLLER_H

#include <vector>
#include <string>
#include <memory>
#include <stdint.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <mw_core/types.h>

namespace controller {

/**
 * @class AbstractController
 * @brief Provides an interface for motion controllers. All controllers written to work as executive plugins must adhere to this interface.
 */
class AbstractController {

 public:

  typedef boost::shared_ptr<::controller::AbstractController> Ptr;

  /**
   * @brief Initialize tf listener
   */
  virtual void initialize(const std::string &name,
                          const TFPtr &tf_listener_ptr) {
    name_ = name;
    tf_listener_ptr_ = tf_listener_ptr;
  };

  /**
   * @brief Virtual destructor
   */
  virtual ~AbstractController() {};

  /**
   * @brief Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base.
   * @param pose The current pose of the robot.
   * @param velocity The current velocity of the robot.
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
   * @param message Optional more detailed outcome as a string
   * @return Result code as described on ExePath action result
   */
  virtual unsigned int computeVelocityCommands(const geometry_msgs::PoseStamped &pose,
                                               const geometry_msgs::TwistStamped &velocity,
                                               geometry_msgs::TwistStamped &cmd_vel,
                                               std::string &message) = 0;

  /**
   * @brief Check if the goal pose has been achieved by the local planner
   * @param angle_tolerance The angle tolerance in which the current pose will be partly accepted as reached goal
   * @param dist_tolerance The distance tolerance in which the current pose will be partly accepted as reached goal
   * @return True if achieved, false otherwise
   */
  virtual bool isGoalReached(double dist_tolerance, double angle_tolerance) = 0;

  /**
   * @brief Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) = 0;

  /**
   * @brief Requests the planner to cancel, e.g. if it takes too much time.
   * @return True if a cancel has been successfully requested, false if not implemented.
   */
  virtual bool cancel() = 0;

  virtual void reset() = 0;

 protected:
  /**
   * @brief Constructor
   */
  AbstractController() {};

  //! shared pointer to the shared tf listener
  TFPtr tf_listener_ptr_;

  //! name of controller
  std::string name_;

};
} /* namespace controller */

#endif //CONTROLLER_EXECUTIVE_BASE_CONTROLLER_H
