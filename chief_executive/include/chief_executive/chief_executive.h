#ifndef CHIEF_EXECUTIVE_H_
#define CHIEF_EXECUTIVE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

#include <mw_msgs/ChiefExecutiveMode.h>
#include <mw_msgs/Route.h>
#include <mw_msgs/Task.h>
#include <mw_msgs/Duty.h>
#include <mw_msgs/GetDuty.h>
#include <mw_msgs/SetDuty.h>
#include <mw_msgs/SetRoute.h>
#include <mw_msgs/GetRouteStatus.h>
#include <mw_msgs/SetTask.h>
#include <mw_msgs/GetTaskStatus.h>
#include <mw_msgs/SetMode.h>
#include <mw_msgs/RobotStatus.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <vector>
#include <string>

/**
 * error codes
 * 0 == all is well
 * 1 == AUTO_MODE sanity broken
 * 2 == unable to process task
 **/

/**
 * TODO
 * 1. take a look at smach architecture 
 * */


namespace executive {

struct DutyMonitor {
  mw_msgs::Duty duty;
  unsigned int current_task_idx = 0;
  bool complete = false;
  bool duty_set = false;
  //will probably add more things here so created a struct
};

class ChiefExecutiveClass {

 public:
  ChiefExecutiveClass();
  ~ChiefExecutiveClass();
  bool getDuty(mw_msgs::GetDuty::Request &req, mw_msgs::GetDuty::Response &resp);
  bool setDefaultDuty(mw_msgs::SetDuty::Request &req, mw_msgs::SetDuty::Response &resp);
  bool setCurrentDuty(mw_msgs::SetDuty::Request &req, mw_msgs::SetDuty::Response &resp);
  bool setCEOMode(mw_msgs::SetMode::Request &req, mw_msgs::SetMode::Response &resp);
  void executeCB(const ros::TimerEvent &event);
  bool sanityForAutoMode();
  bool processTask();
  void resetDutyMonitor();
  void completeEndBehaviour(float duration);
  bool pauseDuty(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

 private:
  ros::NodeHandle nh_;
  bool auto_manual_; //true == auto
  bool start_; //true if start button is pressed
  bool emergency_stop_; //true if e stop is pressed

  std::string default_factory_;

  DutyMonitor current_duty_;
  DutyMonitor default_duty_;

  ros::CallbackQueue executive_queue_;

  ros::ServiceServer get_duty_;
  ros::ServiceServer set_default_duty_;
  ros::ServiceServer set_current_duty_;
  ros::ServiceServer pause_duty_;
  ros::ServiceServer set_ceo_mode_;

  ros::ServiceClient set_route_client_;
  ros::ServiceClient route_status_client_;
  ros::ServiceClient set_task_client_;
  ros::ServiceClient task_status_client_;
  ros::Publisher status_pub_;

  int battery_status_;

  int robot_error_code_;

  int error_code_; //denotes error

  ros::Timer execute_timer_;

  mw_msgs::RobotStatus robot_status_;
  bool auto_pause_;

};
}

#endif
