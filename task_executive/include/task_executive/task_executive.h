/**
 * Author : Sarthak Mittal
 * Task Executive class to handle different tasks (except routes)
 */

#ifndef TASK_EXECUTIVE_H
#define TASK_EXECUTIVE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <mw_msgs/Task.h>
#include <mw_msgs/GetTask.h>
#include <mw_msgs/SetTask.h>
#include <mw_msgs/GetTaskStatus.h>

#include <boost/thread.hpp>

namespace executive {

class TaskExecutiveClass {
 private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::CallbackQueue task_executive_queue_;

  ros::ServiceServer get_task_server_;
  ros::ServiceServer set_task_server_;
  ros::ServiceServer set_state_server_;
  ros::ServiceServer task_status_server_;

  boost::thread *task_thread_;

  mw_msgs::Task current_task_;

  double loop_frequency_;
  bool run_task_;
  bool task_running_;
  bool task_complete_;

 public:
  TaskExecutiveClass();

  ~TaskExecutiveClass();

  bool getTask(mw_msgs::GetTask::Request &req, mw_msgs::GetTask::Response &resp);

  bool setTask(mw_msgs::SetTask::Request &req, mw_msgs::SetTask::Response &resp);

  void taskThread();

  bool taskStatus(mw_msgs::GetTaskStatus::Request &req, mw_msgs::GetTaskStatus::Response &resp);

  void spinClass();

  double getLoopFrequency();

};

};

#endif //TASK_EXECUTIVE_TASK_EXECUTIVE_H
