#include <chief_executive/chief_executive.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "chief_executive_node");

  executive::ChiefExecutiveClass chief_exec;

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return(0);
}
