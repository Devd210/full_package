/**
 * Author : Sarthak Mittal
 * Utility node to read tasks from yaml files and call the task_executive/set_route service
 */

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mw_msgs/SetTask.h>

#include <yaml-cpp/yaml.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "set_task_client_node");

  if (argc != 2) {
    ROS_INFO("usage: set_task_client path/to/task/file");
    return 1;
  }

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<mw_msgs::SetTask>("task_executive/set_task");

  YAML::Node task_file = YAML::LoadFile(argv[1]);

  mw_msgs::SetTask srv;
  srv.request.task.id = task_file["id"].as<unsigned int>();
  srv.request.task.duration = task_file["duration"].as<float>();
  srv.request.task.end_behaviour = task_file["end_behaviour"].as<float>();
  srv.request.task.status = mw_msgs::Task::NOT_STARTED;
  srv.request.task.type = task_file["type"].as<unsigned char>();

  if (srv.request.task.type == 0) {
    YAML::Node route_file = YAML::LoadFile(task_file["route"].as<std::string>());
    srv.request.task.route.id = route_file["id"].as<unsigned int>();
    srv.request.task.route.header.stamp = ros::Time::now();
    srv.request.task.route.header.frame_id = "map";
    for (int i = 0; i < route_file["waypoints"].size(); i++) {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";
      pose.pose.position.x = route_file["waypoints"][i][0].as<double>();
      pose.pose.position.y = route_file["waypoints"][i][1].as<double>();
      pose.pose.position.z = 0;
      srv.request.task.route.waypoints.push_back(pose);
    }
  }

  if (client.call(srv)) {
    ROS_INFO("[set_task_client]: set task from '%s'", argv[1]);
  } else {
    ROS_ERROR("Failed to call service task_executive/set_task");
    return 1;
  }

  return 0;

}
