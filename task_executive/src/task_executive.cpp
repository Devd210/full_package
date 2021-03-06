/**
 * Author : Sarthak Mittal
 * Task Executive class to handle different tasks (except routes)
 */

#include "task_executive/task_executive.h"

namespace executive {

TaskExecutiveClass::TaskExecutiveClass() : private_nh_("~") {
  get_task_server_ = private_nh_.advertiseService("get_current_task", &TaskExecutiveClass::getTask, this);
  set_task_server_ = private_nh_.advertiseService("set_task", &TaskExecutiveClass::setTask, this);
  task_status_server_ = private_nh_.advertiseService("task_status", &TaskExecutiveClass::taskStatus, this);

  loop_frequency_ = 20;
  run_task_ = false;
  task_running_ = false;
  task_complete_ = false;
  
  task_thread_ = new boost::thread(boost::bind(&TaskExecutiveClass::taskThread, this));
}

TaskExecutiveClass::~TaskExecutiveClass() {}

bool TaskExecutiveClass::getTask(mw_msgs::GetTask::Request &req, mw_msgs::GetTask::Response &resp) {
  if (task_running_ && req.id == 0) {
    ROS_INFO("[task_executive]: Get current task service called!");
    resp.task = current_task_;
    return true;
  }
  ROS_WARN("[task_executive]: Invalid task ID requested!");
  return false;
}

bool TaskExecutiveClass::setTask(mw_msgs::SetTask::Request &req, mw_msgs::SetTask::Response &resp) {
  if (task_running_) {
    ROS_INFO("[task_executive]: A task is already running!");
    resp.success = (unsigned char) false;
    return false;
  }

  ROS_INFO("[task_executive]: Set task service called!");
  task_complete_ = false;
  if (req.task.type == mw_msgs::Task::WAIT) {
    //this is a task to wait for 10 seconds
    run_task_ = true;
    current_task_ = req.task;
  }
  resp.success = (unsigned char) true;
  return true;
}

bool TaskExecutiveClass::taskStatus(mw_msgs::GetTaskStatus::Request &req,
                                    mw_msgs::GetTaskStatus::Response &resp) {
  if (!run_task_ && !task_running_ && !task_complete_) {
    resp.status = mw_msgs::Task::NOT_STARTED;
    return true;
  }
  if (run_task_ && task_running_ && !task_complete_) {
    resp.status = mw_msgs::Task::WIP;
    return true;
  }
  if (!run_task_ && !task_running_ && task_complete_) {
    resp.status = mw_msgs::Task::COMPLETED;
    return true;
  }
  return false;
}

void TaskExecutiveClass::taskThread() {
  while (true) {
    while (!run_task_) {
      //waiting for the task to come
    }
    task_running_ = true;
    ros::Duration(current_task_.duration).sleep();
    task_running_ = false;
    task_complete_ = true;
    run_task_ = false;
  }
}

void TaskExecutiveClass::spinClass() {
  task_executive_queue_.callAvailable(ros::WallDuration());
}

double TaskExecutiveClass::getLoopFrequency() {
  return loop_frequency_;
}

}