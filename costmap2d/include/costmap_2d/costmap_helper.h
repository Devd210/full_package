/**
 *
 * @file costmap_helper.h
 * @brief Costmap helper class to support all the costmap related calls and subscriptions
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 *
 */

#pragma once

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <mw_msgs/GetDouble.h>
#include <mw_msgs/GetString.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/thread.hpp>
#include <std_msgs/Float64.h>

#include <vector>
#include <string>
#include <mutex>
#include <memory>

#include <std_msgs/UInt16.h>
#include <mw_msgs/CostGrid.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_srvs/Trigger.h>

namespace costmap_2d
{

class CostmapHelper
{

 public:
  typedef std::shared_ptr<::costmap_2d::CostmapHelper> Ptr;

  CostmapHelper(const ros::NodeHandle &nh, const std::string &ns);

  std::string getGlobalFrameID();
  std::vector<geometry_msgs::Point> getRobotFootprint();
  unsigned int getSizeInCellsX();
  unsigned int getSizeInCellsY();
  double getOriginX();
  double getOriginY();
  double getResolution();
  bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my);
  bool mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy);
  std::vector<unsigned char> costmap_;

  unsigned char getCost(unsigned int x, unsigned int y);

  void setCost(unsigned int x, unsigned int y, unsigned char cost);
  unsigned char *getCharMap();

  bool isReady();

 private:
  ros::NodeHandle nh_;
  unsigned char *char_map_;
  boost::thread *t;

  std::string namespace_;

  std::shared_ptr<std::mutex> access_;
  double circumscribed_radius_;
  unsigned int size_x_, size_y_;
  double resolution_;
  double origin_x_, origin_y_;
  std::string costmap_global_frame_;

  ros::Subscriber origin_x_sub_;
  ros::Subscriber origin_y_sub_;
  ros::Subscriber resolution_sub_;
  ros::Subscriber size_x_sub_;
  ros::Subscriber size_y_sub_;
  ros::Subscriber cost_grid_sub_;
  ros::Subscriber global_frame_id_sub_;
  ros::Subscriber footprint_sub_;

  void originXCB(std_msgs::Float64 msg);
  void originYCB(std_msgs::Float64 msg);
  void resolutionCB(std_msgs::Float64 msg);
  void sizeXCB(std_msgs::UInt16 msg);
  void sizeYCB(std_msgs::UInt16 msg);
  void costGridCB(mw_msgs::CostGrid msg);
  void globalFrameCB(std_msgs::String msg);
  void footprintCB(geometry_msgs::PolygonStamped msg);
  void start();
  void stop();
  void resetLayers();
  void pause();
  void resume();

  std::vector<geometry_msgs::Point> footprint_;
  std::vector<unsigned char> cost_grid_;

  ros::ServiceClient start_srv_client_;
  ros::ServiceClient stop_srv_client_;
  ros::ServiceClient reset_srv_client_;
  ros::ServiceClient pause_srv_client_;
  ros::ServiceClient resume_srv_client_;

};

} // namespace costmap_2d
