/**
 *
 * @file costmap_client.h
 * @brief Costmap2DClient class.
 * @author Sarthak Mittal <sarthak.mittal@mowito.in>
 *
 */

#ifndef MAP_EXPLORATION__COSTMAP_CLIENT_H_
#define MAP_EXPLORATION__COSTMAP_CLIENT_H_

#include <ros/ros.h>
#include <mw_core/types.h>
#include <mw_core/utility_functions.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Pose.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>

namespace map_exploration {

class Costmap2DClient {
 public:
  /**
   * @brief Contructs client and start listening
   * @details Constructor will block until first map update is received and
   * map is ready to use, also will block before trasformation
   * robot_base_frame <-> global_frame is available.
   *
   * @param param_nh node hadle to retrieve parameters from
   * @param subscription_nh node hadle where topics will be subscribed
   * @param tf_listener Will be used for transformation of robot pose.
   */
  Costmap2DClient(ros::NodeHandle &param_nh,
                  ros::NodeHandle &subscription_nh,
                  const TFPtr &tf_listener);
  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @return pose of the robot in the global frame of the costmap
   */
  geometry_msgs::Pose getRobotPose() const;

  /**
   * @brief Return a pointer to the "master" costmap which receives updates from
   * all the layers.
   *
   * This pointer will stay the same for the lifetime of Costmap2DClient object.
   */
  costmap_2d::Costmap2D *getCostmap() {
    return &costmap_;
  }

  /**
   * @brief Return a pointer to the "master" costmap which receives updates from
   * all the layers.
   *
   * This pointer will stay the same for the lifetime of Costmap2DClient object.
   */
  const costmap_2d::Costmap2D *getCostmap() const {
    return &costmap_;
  }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  const std::string &getGlobalFrameID() const {
    return global_frame_;
  }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  const std::string &getBaseFrameID() const {
    return robot_base_frame_;
  }

 protected:
  void updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void updatePartialMap(const map_msgs::OccupancyGridUpdate::ConstPtr &msg);

  costmap_2d::Costmap2D costmap_;

  TFPtr tf_listener_;

  /// point clouds
  std::string global_frame_;      ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
  double tf_timeout_;    ///< timeout before transform errors

 private:
  // will be unsubscribed at destruction
  ros::Subscriber costmap_sub_;
  ros::Subscriber costmap_updates_sub_;
};

}  // namespace map_exploration

#endif  // MAP_EXPLORATION__COSTMAP_CLIENT_H_
