#ifndef TASK_EXECUTIVE_H_
#define TASK_EXECUTIVE_H_

#include <ros/ros.h>
#include <diagnostic/local_diagnostic.h>
#include <mw_msgs/SetNodeState.h>

#include "task_executive/task_executive.h"

bool setDiagnosticState(mw_msgs::SetNodeState::Request &req,
                        mw_msgs::SetNodeState::Response &resp,
                        boost::shared_ptr<diagnosticWatch> &taskExecutiveWatch) {
  // node calling this service is responsible for ensuring that correct input is being provided
  if (req.state) {
    taskExecutiveWatch->process_event(activeInterrupt());
  } else {
    taskExecutiveWatch->process_event(pauseInterrupt());
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "task_executive_node");

  ros::NodeHandle nh("task_executive_node");

  // setting up the diagnostics
  boost::shared_ptr<diagnosticWatch> taskExecutiveWatch;
  taskExecutiveWatch.reset(new diagnosticWatch("task_executive_node"));

  // dianostic services
  ros::ServiceServer setState = nh.advertiseService<mw_msgs::SetNodeState::Request, mw_msgs::SetNodeState::Response>
      ("set_node_state", boost::bind(&setDiagnosticState, _1, _2, taskExecutiveWatch));
  taskExecutiveWatch->initiate();

  executive::TaskExecutiveClass task_exec;

  ros::Rate loop_rate(task_exec.getLoopFrequency());

  while (ros::ok()) {
    // if the node is in active state, execute the queue
    if (taskExecutiveWatch->getState()) {
      ros::spinOnce();
      task_exec.spinClass();
    }
      // else if in pause state, do NOT execute the queue
    else {
      ros::spinOnce();
    }
    loop_rate.sleep();
  }

  return 0;
}

#endif