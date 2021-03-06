/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.hpp>
#include <tf2/LinearMath/Transform.h>
#include <std_srvs/Empty.h>
#include <mw_msgs/GetDouble.h>
#include <mw_msgs/GetString.h>
#include <mw_msgs/GetPose.h>
#include <mw_msgs/SetCost.h>
#include <mw_msgs/GetCircumscribedRadius.h>
#include <mw_msgs/ComputeCircumscribedCost.h>
#include <costmap_2d/inflation_layer.h>
#include <cstdio>
#include <algorithm>
#include <vector>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mw_msgs/Route.h>
#include <std_msgs/Float64.h>
#include <mw_msgs/CostGrid.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Trigger.h>

class SuperValue : public XmlRpc::XmlRpcValue {
 public:
  void setStruct(XmlRpc::XmlRpcValue::ValueStruct *a) {
    _type = TypeStruct;
    _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
  }
  void setArray(XmlRpc::XmlRpcValue::ValueArray *a) {
    _type = TypeArray;
    _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
  }
};

namespace costmap_2d {
bool callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

/** @brief A ROS wrapper for a 2D Costmap. Handles subscribing to
* topics that provide observations about obstacles in either the form
* of PointCloud or LaserScan messages. */
class Costmap2DROS {
 public:

  typedef std::shared_ptr<::costmap_2d::Costmap2DROS> Ptr;

  /**
   * @brief  Constructor for the wrapper
   * @param name The name for this costmap
   * @param tf A reference to a TransformListener
   */
  Costmap2DROS(const std::string &name, tf2_ros::Buffer &tf);
  ~Costmap2DROS();

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  void start();

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  void stop();

  /**
   * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
   */
  void pause();

  /**
   * @brief  Resumes costmap updates
   */
  void resume();
  /**
   * @brief Updates the map as the robot moves around
   */
  void updateMap();

  /**
   * @brief Reset each individual layer
   */
  void resetLayers();

  /** @brief Same as getLayeredCostmap()->isCurrent(). */
  bool isCurrent() {
    return layered_costmap_->isCurrent();
  }

  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
   * @return True if the pose was set successfully, false otherwise
   */
  bool getRobotPose(geometry_msgs::PoseStamped &global_pose) const;

  /** @brief Returns costmap name */
  std::string getName() const {
    return name_;
  }

  /** @brief Returns the delay in transform (tf) data that is tolerable in seconds */
  double getTransformTolerance() const {
    return transform_tolerance_;
  }

  /** @brief Return a pointer to the "master" costmap which receives updates from all the layers.
   *
   * Same as calling getLayeredCostmap()->getCostmap(). */
  Costmap2D *getCostmap() {
    return layered_costmap_->getCostmap();
  }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  std::string getGlobalFrameID() {
    return global_frame_;
  }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  std::string getBaseFrameID() {
    return robot_base_frame_;
  }
  LayeredCostmap *getLayeredCostmap() {
    return layered_costmap_;
  }

  /** @brief Returns the current padded footprint as a geometry_msgs::Polygon. */
  geometry_msgs::Polygon getRobotFootprintPolygon() {
    return costmap_2d::toPolygon(padded_footprint_);
  }

  /** @brief Return the current footprint of the robot as a vector of points.
   *
   * This version of the footprint is padded by the footprint_padding_
   * distance, set in the rosparam "footprint_padding".
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::Point> getRobotFootprint() {
    return padded_footprint_;
  }

  /** @brief Return the current unpadded footprint of the robot as a vector of points.
   *
   * This is the raw version of the footprint without padding.
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::Point> getUnpaddedRobotFootprint() {
    return unpadded_footprint_;
  }

  /**
   * @brief  Build the oriented footprint of the robot at the robot's current pose
   * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
   */
  void getOrientedFootprint(std::vector<geometry_msgs::Point> &oriented_footprint) const;

  /** @brief Set the footprint of the robot to be the given set of
   * points, padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point> &points);

  /** @brief Set the footprint of the robot to be the given polygon,
   * padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon &footprint);

  //Adding callback for services,these functions in-turn call the costmap2DROS functions

  bool startCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool stopCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool pauseCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool resumeCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool resetLayersCB(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res);
  bool getRobotPoseCB(mw_msgs::GetPose::Request &req, mw_msgs::GetPose::Response &res);
  bool getNameCB(mw_msgs::GetString::Request &req, mw_msgs::GetString::Response &res);
  bool getTransformToleranceCB(mw_msgs::GetDouble::Request &req, mw_msgs::GetDouble::Response &res);
  bool setCostCB(mw_msgs::SetCost::Request &req, mw_msgs::SetCost::Response &res);
  bool getCircumscribedRadiusCB(mw_msgs::GetCircumscribedRadius::Request &req,
                                mw_msgs::GetCircumscribedRadius::Response &res);
  bool computeCircumscribedCostCB(mw_msgs::ComputeCircumscribedCost::Request &req,
                                  mw_msgs::ComputeCircumscribedCost::Response &res);
  void spinClass();

  void cost_grid_pub_thread();
  ros::Publisher origin_x_pub_, origin_y_pub_, resolution_pub_, cost_grid_pub_, global_frame_id_pub_,base_frame_id_pub_,size_x_pub_,size_y_pub_;
  boost::thread *t;

 protected :
  LayeredCostmap *layered_costmap_;
  std::string name_;
  tf2_ros::Buffer &tf_;  ///< @brief Used for transforming point clouds
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string robot_base_frame_;  ///< @brief The frame_id of the robot base
  double transform_tolerance_;  ///< timeout before transform errors

 private:
  void pub_thread();

  /** @brief Set the footprint from the new_config object.
   *
   * If the values of footprint and robot_radius are the same in
   * new_config and old_config, nothing is changed. */
  void readFootprintFromConfig(const costmap_2d::Costmap2DConfig &new_config,
                               const costmap_2d::Costmap2DConfig &old_config);

  void loadOldParameters(ros::NodeHandle &nh);
  void warnForOldParameters(ros::NodeHandle &nh);
  void checkOldParam(ros::NodeHandle &nh, const std::string &param_name);
  void copyParentParameters(const std::string &plugin_name, const std::string &plugin_type, ros::NodeHandle &nh);
  void reconfigureCB(costmap_2d::Costmap2DConfig &config, uint32_t level);
  void movementCB(const ros::TimerEvent &event);
  void mapUpdateLoop(double frequency);

  ros::CallbackQueue callback_queue_;
  double frequency_;
  bool map_update_thread_shutdown_;
  bool stop_updates_, initialized_, stopped_, robot_stopped_;
  boost::thread *map_update_thread_;  ///< @brief A thread for updating the map
  ros::Timer timer_;
  ros::Time last_publish_;
  ros::Duration publish_cycle;
  pluginlib::ClassLoader<Layer> plugin_loader_;
  geometry_msgs::PoseStamped old_pose_;
  Costmap2DPublisher *publisher_;
  dynamic_reconfigure::Server<costmap_2d::Costmap2DConfig> *dsrv_;

  boost::recursive_mutex configuration_mutex_;

  ros::Subscriber footprint_sub_;
  ros::Publisher footprint_pub_;
  std::vector<geometry_msgs::Point> unpadded_footprint_;
  std::vector<geometry_msgs::Point> padded_footprint_;
  float footprint_padding_;
  costmap_2d::Costmap2DConfig old_config_;
  costmap_2d::Costmap2D *master_costmap_;

  //The service servers to provide all the costmap services over the wire

  ros::ServiceServer start_;
  ros::ServiceServer stop_;
  ros::ServiceServer pause_;
  ros::ServiceServer resume_;
  ros::ServiceServer updateMap_;
  ros::ServiceServer resetLayers_;
  ros::ServiceServer getRobotPose_;
  ros::ServiceServer getName_;
  ros::ServiceServer getTransformTolerance_;
  ros::ServiceServer setCost_;
  ros::ServiceServer getCharMap_;
  ros::ServiceServer getCircumscribedRadius_;
  ros::ServiceServer computeCircumscribedCost_;
};
// class Costmap2DROS
}  // namespace costmap_2d
