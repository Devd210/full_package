#include <mw_msgs/GetPathAction.h>
#include <mw_msgs/ExePathAction.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>
#include <mw_core/utility_functions.h>
#include <actionlib/client/simple_action_client.h>
#include <queue>

class SBPLPlanTest
{

public:
  SBPLPlanTest() : private_nh_("~"),
                   planner_action_("/global_planner/get_path", true),
                   controller_action_("/controller_executive/exe_path", true)
  {
    std::string planner;
    private_nh_.param("planner", planner, std::string("NavfnPlanner"));
    private_nh_.param("run_controller", run_controller_, false);
    plan_pub_ = nh_.advertise<nav_msgs::Path>("/path", 2);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    planner_action_.waitForServer();
    ROS_INFO("Planner Action Server is up and running");

    controller_action_.waitForServer();
    ROS_INFO("Controller is up and running");

    goal_.use_current_pose = true;
    goal_.tolerance = 0.5;
    goal_.planner = planner;

    exe_path_goal_.controller = "trajectory_planner";
    exe_path_goal_.controller_execution = "trajectory_planner_execution";

    concurrency_slot_ = 0;

    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &SBPLPlanTest::goalCB, this);
    t = boost::thread(&SBPLPlanTest::execute, this);
  }

  void donePlannerCB(const actionlib::SimpleClientGoalState &state, const mw_msgs::GetPathResultConstPtr &result)
  {
    ROS_INFO("Action has finished with status: %s", result->message.c_str());
    if (!result->path.poses.empty())
    {
      ROS_INFO("The cost of the computed path is %lf", result->cost);
      plan_ = result->path;
      plan_pub_.publish(result->path);
      call_controller_ = true;
    }
    else
    {
      ROS_INFO("Empty plan received! Won't be calling controller");
      call_controller_ = false;
    }
    running_ = false;
    ROS_DEBUG("Lets go baby!");
  }

  void activePlannerCB()
  {
    ROS_INFO("Plan requested, lets see how it goes");
  }

  void feedbackPlannerCB(const mw_msgs::GetPathFeedbackConstPtr &feedback)
  {
    ROS_INFO("Planner is planning!");
  }

  void goalCB(const geometry_msgs::PoseStamped::Ptr &msg)
  {
    target_.pose.position.x = msg->pose.position.x;
    target_.pose.position.y = msg->pose.position.y;
    target_.pose.position.z = msg->pose.position.z;
    target_.pose.orientation.x = msg->pose.orientation.x;
    target_.pose.orientation.y = msg->pose.orientation.y;
    target_.pose.orientation.z = msg->pose.orientation.z;
    target_.pose.orientation.w = msg->pose.orientation.w;
    target_.header = msg->header;

    goal_.waypoints.clear();
    goal_.waypoints.push_back(target_);

    goal_.concurrency_slot = concurrency_slot_;
    goalQueue_.push(goal_);
    ROS_DEBUG("Goal pushed to the queue");
  }

  void execute()
  {
    while (ros::ok())
    {
      while (goalQueue_.empty())
      {
        ROS_DEBUG("Queue is empty, going into wait state");
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
      }
      mw_msgs::GetPathGoal goal = goalQueue_.front();
      goalQueue_.pop();

      running_ = true;
      planner_action_.sendGoal(goal,
                               boost::bind(&SBPLPlanTest::donePlannerCB, this, _1, _2),
                               boost::bind(&SBPLPlanTest::activePlannerCB, this),
                               boost::bind(&SBPLPlanTest::feedbackPlannerCB, this, _1));

      while (ros::ok() && running_)
        ;

      if (call_controller_ && run_controller_)
      {
        exe_path_goal_.path = plan_;
        exe_path_goal_.concurrency_slot = concurrency_slot_;
        controller_action_.sendGoal(exe_path_goal_,
                                    boost::bind(&SBPLPlanTest::doneControllerCB, this, _1, _2),
                                    boost::bind(&SBPLPlanTest::activeControllerCB, this),
                                    boost::bind(&SBPLPlanTest::feedbackControllerCB, this, _1));
        controller_action_.waitForResult();
      }
      ROS_INFO("The controller has finished");
      concurrency_slot_++;
    }
  }

  void doneControllerCB(const actionlib::SimpleClientGoalState &state, const mw_msgs::ExePathResultConstPtr &result)
  {
    ROS_INFO("The controller has finished with status %s", state.toString().c_str());
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    cmd_vel_pub_.publish(cmd_vel);
    ROS_INFO("Published an explicit zero velocity");
  }

  void activeControllerCB()
  {
    ROS_INFO("Controller has received the global plan, let's see how it goes!");
  }

  void feedbackControllerCB(const mw_msgs::ExePathFeedbackConstPtr &feedback)
  {
    ROS_DEBUG("Received feedback from controller with message: %s", feedback->message.c_str());
    cmd_vel_pub_.publish(feedback->last_cmd_vel.twist);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher plan_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber goal_sub_;

  actionlib::SimpleActionClient<mw_msgs::GetPathAction> planner_action_;
  actionlib::SimpleActionClient<mw_msgs::ExePathAction> controller_action_;

  mw_msgs::GetPathGoal goal_;
  mw_msgs::ExePathGoal exe_path_goal_;

  nav_msgs::Path plan_;

  bool running_;
  bool call_controller_;

  geometry_msgs::PoseStamped target_;
  unsigned char concurrency_slot_;
  std::queue<mw_msgs::GetPathGoal> goalQueue_;
  boost::thread t;

  bool run_controller_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sbpl_test_node");
  SBPLPlanTest test;
  ros::spin();
  return 0;
}