#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <mw_msgs/Diagnostic.h>


namespace sc = boost::statechart;
/**
struct errorInterrupt : sc::event< errorInterrupt > {
    public:
        errorInterrupt(int code): error_(code){}
        int error_;
};

struct pauseInterrupt : sc::event<pauseInterrupt>{};
struct activeInterrupt : sc::event<activeInterrupt>{};

struct resetInterrupt : sc::event< resetInterrupt > {};

struct Active;
struct Pause;
struct diagnosticWatch : sc::state_machine< diagnosticWatch, Active > {};

struct Error;
struct Pause;

struct allGood;

struct Active : sc::simple_state<Active, diagnosticWatch, allGood>{
    public:
        typedef mpl::list< sc::transition< resetInterrupt, Active>, sc::transition<pauseInterrupt, Pause> >reactions;
        Active(std::string node_name);
        ~Active(){}
        void setErrorStatus(const errorInterrupt& e);
        void statusCB(const ros::TimerEvent& event);
        //void setErrorStatus(int error){error_status_ = error;}
    private:
        int error_status_ = 0;
        ros::Timer status_timer_;
        ros::Publisher status_publisher_;

};

struct Pause : sc::simple_state<Pause, diagnosticWatch>{

    public:
        typedef sc::transition<activeInterrupt, Active> reactions;

};

struct allGood : sc::simple_state< allGood, Active > {
    typedef sc::transition< errorInterrupt, Error, Active, &Active::setErrorStatus > reactions;
};
struct Error : sc::simple_state< Error, Active > {};

**/

struct allGood;

struct errorInterrupt : sc::event< errorInterrupt > {
    public:
        errorInterrupt(int code): error_(code){}
        int error_;
};
struct resetInterrupt : sc::event< resetInterrupt >{};
struct pauseInterrupt : sc::event< pauseInterrupt >{};
struct activeInterrupt : sc::event< activeInterrupt >{};

struct Active;
//struct allGood;
struct diagnosticWatch : sc::state_machine< diagnosticWatch, Active > {
    public:
        diagnosticWatch(std::string node_name);
        ~diagnosticWatch(){}
        void statusCB(const ros::TimerEvent& event);
        void setState(const pauseInterrupt& p);
        void setState(const activeInterrupt& a);
        bool getState();
        void setErrorCode(int error);
    private:
        std::string node_name_;
        bool state_;
        int error_status_ = 0;
        ros::Timer status_timer_;
        ros::Publisher status_publisher_;
};
struct Pause;
struct Error;

struct Active : sc::simple_state<Active, diagnosticWatch, allGood>{
    public:
        typedef sc::transition<pauseInterrupt, Pause, diagnosticWatch, &diagnosticWatch::setState> reactions;
        //Active(){}
        //~Active(){}
        void setErrorCode(const errorInterrupt& e);
        void resetErrorCode(const resetInterrupt& r);
    //private:
        //int error_code_;

};

struct Pause : sc::simple_state<Pause, diagnosticWatch>{
    public: 
        typedef sc::transition<activeInterrupt, Active, diagnosticWatch, &diagnosticWatch::setState> reactions;
};

struct allGood : sc::simple_state<allGood, Active>{
    public:
        typedef sc::transition<errorInterrupt, Error ,Active, &Active::setErrorCode> reactions;
};

struct Error : sc::simple_state<Error, Active>{
    public:
        typedef sc::transition<resetInterrupt, allGood ,Active, &Active::resetErrorCode> reactions;
};
