/**
*
* @file clear_costmap_recovery.h
* @brief clear costmap recovery class
* @author Sarthak Mittal <sarthak.mittal@mowito.in>
*
*/

#include <recovery_executive/plugins/clear_costmap_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

PLUGINLIB_EXPORT_CLASS(recovery::ClearCostmapRecovery, recovery::AbstractRecovery)

namespace recovery {

ClearCostmapRecovery::ClearCostmapRecovery(): initialized_(false) {}

void ClearCostmapRecovery::initialize(const std::string &name, const TFPtr &tf_listener_ptr) {
  if(!initialized_) {
    name_ = name;
    tf_listener_ptr_ = tf_listener_ptr;
    ros::NodeHandle nh;
    reset_costmap_srv_ = nh.serviceClient<std_srvs::Trigger>("/costmap/local_costmap/reset_layers");
    initialized_ = true;
  }
  else{
    ROS_ERROR("[Clear Costmap Recovery]: You should not call initialize twice on this object, doing nothing");
  }
}

uint32_t ClearCostmapRecovery::runBehavior(std::string &message) {
  if(!initialized_){
    ROS_ERROR("[Clear Costmap Recovery]: This object must be initialized before runBehavior is called");
    return 1;
  }

  std_srvs::Trigger reset_srv;
  if (reset_costmap_srv_.call(reset_srv)) {
    if (reset_srv.response.success) {
      ROS_INFO("[Clear Costmap Recovery]: %s", reset_srv.response.message.c_str());
      return 0;
    } else {
      ROS_WARN("[Clear Costmap Recovery]: Failed to reset the local costmap!");
      return 1;
    }
  }

  return 1;
}

bool ClearCostmapRecovery::cancel() {
  return false;
}

};
