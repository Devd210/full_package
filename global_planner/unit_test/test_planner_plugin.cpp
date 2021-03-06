/**
 * @file test_planner_plugin.cpp
 * @brief Used to test various planners conforming to the AbstractPlanner defination
 * @author <sourav.agrawal@mowito.in>
 */

#include <string>
#include <global_planner/base_planner.h>
#include <queue>
#include <mw_msgs/GetPathAction.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <tf2/utils.h>
#include <pluginlib/class_loader.h>
#include <random>

enum mode
{
    RVIZ,
    RANDOM,
    WAYPOINTS
};

namespace global_planner
{
class PlannerTest
{
public:
    PlannerTest(char *file_name);
    ~PlannerTest(){}
    bool threadJoin(){
        if(t.timed_join(boost::posix_time::milliseconds(100))){
            return true;
        }
        return false;
    }
private:
    void parseYAML(char *file_name);
    void randGenerator();
    void goalCB(geometry_msgs::PoseStamped msg);
    void exeQueue();
    //Planner Name
    std::string planner_name_;
    std::string planner_type_;
    //For random test
    unsigned int num_of_tests_;
    double limit_x_min_, limit_x_max_, limit_y_min_, limit_y_max_;
    //The queue that will be executed
    std::queue<geometry_msgs::PoseStamped> goal_queue_;
    //The thread that will execute the queue
    boost::thread t;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_;
    std::vector<geometry_msgs::PoseStamped> plan_;
    mode cur_mode_;
    boost::mutex queue_mtx_;
    YAML::Node file_;
    pluginlib::ClassLoader<global_planner::AbstractPlanner> planner_loader_;
    boost::shared_ptr<global_planner::AbstractPlanner> planner_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
};

PlannerTest::PlannerTest(char *file_name) : planner_loader_("global_planner", "global_planner::AbstractPlanner")
{
    ros::NodeHandle private_nh("~");
    try
    {
        file_ = YAML::LoadFile(file_name);
    }
    catch (YAML::BadFile &e)
    {
        ROS_FATAL("Invalid YAML file passed");
        exit(2);
    }
    int mod = file_["mode"].as<unsigned int>();
    switch (mod)
    {
    case 0:
        cur_mode_ = RVIZ;
        break;
    case 1:
        cur_mode_ = RANDOM;
        break;
    case 2:
        cur_mode_ = WAYPOINTS;
        break;
    default:
        ROS_WARN("Invalid mode specified, defaulting to RVIZ mode");
        cur_mode_ = RVIZ;
        break;
    }
    planner_name_ = file_["planner"]["name"].as<std::string>();
    planner_type_ = file_["planner"]["type"].as<std::string>();

    try
    {
        planner_ = planner_loader_.createInstance(planner_type_);
        planner_->initialize(planner_name_);
        ROS_INFO("Plugin Successfully loaded");
    }
    catch (pluginlib::PluginlibException &ex)
    {
        ROS_ERROR("The plugin has failed to load.Error: %s", ex.what());
        exit(3);
    }

    if (cur_mode_ == RANDOM)
    {
        num_of_tests_ = file_["num_of_tests"].as<unsigned int>();
        limit_x_min_ = file_["x_range"][0].as<double>();
        limit_x_max_ = file_["x_range"][1].as<double>();
        limit_y_min_ = file_["y_range"][0].as<double>();
        limit_y_max_ = file_["y_range"][1].as<double>();
        randGenerator();
    }
    else if (cur_mode_ == WAYPOINTS)
    {
        geometry_msgs::PoseStamped temp_pose;
        tf2::Quaternion temp_quat;
        for (int i = 0; i < file_["waypoints"].size(); i++)
        {
            temp_pose.header.frame_id = "map";
            temp_pose.header.seq = i;
            temp_pose.pose.position.x = file_["waypoints"][i][0].as<double>();
            temp_pose.pose.position.y = file_["waypoints"][i][1].as<double>();
            temp_quat.setRPY(0.0, 0.0, file_["waypoints"][i][2].as<double>());
            temp_pose.pose.orientation = tf2::toMsg(temp_quat);
            waypoints_.push_back(temp_pose);
        }
    }
    else if (cur_mode_ == RVIZ)
    {
        goal_sub_ = private_nh.subscribe("/move_base_simple/goal", 5, &PlannerTest::goalCB, this);
    }
    path_pub_ = private_nh.advertise<nav_msgs::Path>("/path",5);
    t = boost::thread(&PlannerTest::exeQueue,this);
}

void PlannerTest::randGenerator()
{
    ROS_DEBUG("Genrating %d random goals", num_of_tests_);
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist_x(limit_x_min_, limit_y_max_);
    std::uniform_real_distribution<double> dist_y(limit_x_min_, limit_y_max_);
    std::uniform_real_distribution<double> dist_ang(-3.14, 3.14);
    geometry_msgs::PoseStamped temp_pose;
    tf2::Quaternion temp_quat;
    for (unsigned int i = 0; i < num_of_tests_; i++)
    {
        temp_pose.header.frame_id = "map";
        temp_pose.header.seq = i;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.pose.position.x = dist_x(mt);
        temp_pose.pose.position.y = dist_y(mt);
        temp_quat.setRPY(0.0, 0.0, dist_ang(mt));
        temp_pose.pose.orientation = tf2::toMsg(temp_quat);
        queue_mtx_.lock();
        goal_queue_.push(temp_pose);
        queue_mtx_.unlock();
    }
}

void PlannerTest::goalCB(geometry_msgs::PoseStamped msg)
{
    queue_mtx_.lock();
    goal_queue_.push(msg);
    queue_mtx_.unlock();
}

void PlannerTest::exeQueue()
{
    ROS_DEBUG("Starting the planning thread");
    geometry_msgs::Quaternion myquat;
    tf2::Quaternion quat;
    geometry_msgs::PoseStamped start;
    double cost;
    unsigned int count = 0;
    std::string msg;
    start.pose.position.x = 0.0;
    start.pose.position.y = 0.0;
    start.pose.position.z = 0.0;
    quat.setRPY(0, 0, 0);
    myquat = tf2::toMsg(quat);
    start.pose.orientation = myquat;
    start.header.frame_id = "map";
    waypoints_.push_back(start);
    ROS_INFO("Waiting for rviz to subscribe to /path");
    while(path_pub_.getNumSubscribers()==0)
                ros::Duration(0.1).sleep();
    ROS_INFO("Wait complete");
    while (ros::ok)
    {
        if (cur_mode_ == WAYPOINTS)
        {
            planner_->makePlan(waypoints_, 0.5, plan_, cost, msg);
            if (plan_.size() > 0)
                ROS_INFO("Plan Computation successfull, the size of plan returned is %ld", plan_.size());
            else
                ROS_INFO("A path coudn't be computed for a given set of waypoints");
            if(path_pub_.getNumSubscribers()>0)
            {
                ROS_DEBUG("Number of subscribers %d",path_pub_.getNumSubscribers());
                path_.header.frame_id = "map";
                path_.header.seq = 0;
                path_.header.stamp = ros::Time::now();
                path_.poses = plan_;
                path_pub_.publish(path_);
            }
            break;
        }
        if (goal_queue_.empty() && cur_mode_ == RVIZ)
        {
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        }
        else if (goal_queue_.empty())
        {
            ROS_INFO("All goals were successfully completed");
            break;
        }
        else
        {
            if (waypoints_.size() > 1)
                waypoints_.pop_back();
            queue_mtx_.lock();
            waypoints_.push_back(goal_queue_.front());
            goal_queue_.pop();
            ROS_DEBUG("size of queue %ld && size of waypoints %ld",goal_queue_.size(),waypoints_.size());
            queue_mtx_.unlock();
            plan_.clear();
            planner_->makePlan(waypoints_, 0.5, plan_, cost, msg);
            if (plan_.size() > 0)
                ROS_INFO("Plan Computation successfull, the size of plan returned is %ld", plan_.size());
            else
            {
                ROS_INFO("Planning failed for goal point (%.3lf,%.3lf)", waypoints_.back().pose.position.x, waypoints_.back().pose.position.y);
            }
            path_.header.frame_id = "map";
            path_.header.seq = count++;
            path_.header.stamp = ros::Time::now();
            path_.poses = plan_;
            path_pub_.publish(path_);
            if(cur_mode_==RANDOM)
                ros::Duration(0.5).sleep();
        }
    }
    ROS_INFO("Planner thread execution is complete");
}
} // namespace global_planner

//The main function
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "planner_test");
    if (argc != 2)
    {
        ROS_FATAL("Invalid use of arguments, test_planner_plugin <yaml filename>");
        return -1;
    }
    global_planner::PlannerTest planner(argv[1]);
    ros::Rate rate(10.0);
    while (ros::ok())
    {
        ros::spinOnce();
        if(planner.threadJoin())
            return 0;
        rate.sleep();
    }
}