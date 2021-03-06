#include <diagnostic/local_diagnostic.h>

namespace sc = boost::statechart;

/*Active::Active(std::string node_name){
    ros::NodeHandle nh;
    status_publisher_ = nh.advertise<std_msgs::Int32>("/error_code", 100);
    status_timer_ = nh.createTimer(ros::Duration(0.1), boost::bind(&Active::statusCB, this, _1));
}

void Active::setErrorStatus(const errorInterrupt& e){
    error_status_ = e.error_;
}

void Active::statusCB(const ros::TimerEvent& event){
    std_msgs::Int32 error;
    error.data = error_status_;
    status_publisher_.publish(error);
}
*/

diagnosticWatch::diagnosticWatch(std::string name){
    node_name_ = name;
    ros::NodeHandle nh(node_name_);
    state_ = true;
    error_status_ = 0;
    status_publisher_ = nh.advertise<mw_msgs::Diagnostic>("diagnostic", 100);
    status_timer_ = nh.createTimer(ros::Duration(0.1), boost::bind(&diagnosticWatch::statusCB, this, _1));
}

void diagnosticWatch::statusCB(const ros::TimerEvent& event){
    mw_msgs::Diagnostic diag;
    diag.error_code = error_status_;
    diag.status = state_;
    status_publisher_.publish(diag);
}
void diagnosticWatch::setState(const pauseInterrupt& p){
    state_ = false;
}

void diagnosticWatch::setState(const activeInterrupt& a){
    state_ = true;
}
bool diagnosticWatch::getState(){
    if(state_){
        return true;
    }else{
        return false;
    }

}

void diagnosticWatch::setErrorCode(int error){
    error_status_ = error;
}

void Active::setErrorCode(const errorInterrupt& e){
    context<diagnosticWatch>().setErrorCode(e.error_);
}

void Active::resetErrorCode(const resetInterrupt& r){
    int error =0;
    context<diagnosticWatch>().setErrorCode(error);
}

