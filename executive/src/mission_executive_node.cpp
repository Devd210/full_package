#include <executive/mission_executive.h>
#include <mw_msgs/SetNodeState.h>
#include <mlicense/license.hpp>

bool setDiagnosticState(mw_msgs::SetNodeState::Request &req,
                        mw_msgs::SetNodeState::Response &resp,
                        boost::shared_ptr<diagnosticWatch> &executiveWatch) {
  //node calling this service is responsible for ensuring that correct input is being provided
  if (req.state) {
    executiveWatch->process_event(activeInterrupt());
  } else {
    executiveWatch->process_event(pauseInterrupt());
  }
  return true;
}

int main(int argc, char **argv) {
  // UNCOMMENT LINES BELOW FOR RELEASE VERSION
  // License lic;
  // if(!lic.verifyLicense()){
  //    ROS_ERROR("[Executive]: License not verified. Exiting");
  //    ros::shutdown();
  // }
  ROS_INFO("[Executive]: License verified");

  ros::init(argc, argv, "mission_executive");
  ros::NodeHandle nh("~");

  // setting up the diagnostics
  boost::shared_ptr<diagnosticWatch> missionExecutiveWatch;
  missionExecutiveWatch.reset(new diagnosticWatch("mission_executive"));

  // dianostic services
  ros::ServiceServer setState = nh.advertiseService<mw_msgs::SetNodeState::Request, mw_msgs::SetNodeState::Response>(
      "set_node_state",
      boost::bind(&setDiagnosticState, _1, _2, missionExecutiveWatch));
  missionExecutiveWatch->initiate();

  mission_executive::MissionExecutive exec;

  ros::Rate loop_rate(exec.getLoopFrequency());
    
  while (ros::ok()) {
    
    exec.startMissionExecutiveStatusPublisher();
    
    // if the node is in active state, execute the queue
    if (missionExecutiveWatch->getState()) {
      ros::spinOnce();
      exec.spinClass();
    }
      // else if in pause state, do NOT execute the queue
    else {
      ros::spinOnce();
    }
    loop_rate.sleep();
  }

  return (0);
}
