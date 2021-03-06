/**
 * Author : Sarthak Mittal
 * Utility node to read duties from yaml files and call the chief_executive/set_current_duty service
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <mw_msgs/Task.h>
#include <mw_msgs/SetDuty.h>
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "set_duty_client_node");

  if (argc != 2) {
    ROS_INFO("usage: set_duty_client path/to/duty/file");
    return 1;
  }

  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<mw_msgs::SetDuty>("chief_executive/set_current_duty");

  try {
    YAML::Node duty_file = YAML::LoadFile(argv[1]);
    mw_msgs::SetDuty srv;
    srv.request.duty.id = duty_file["id"].as<unsigned int>();
    srv.request.duty.total_tasks = duty_file["total_tasks"].as<unsigned long>();
    srv.request.duty.loop_enabled = (unsigned char) duty_file["loop_enabled"].as<bool>();

    for (int i = 0; i < srv.request.duty.total_tasks; i++) {
      try {
        YAML::Node task_file = YAML::LoadFile(duty_file["task" + std::to_string(i + 1)].as<std::string>());

        mw_msgs::Task task;
        task.id = task_file["id"].as<unsigned int>();
        task.duration = task_file["duration"].as<float>();
        task.end_behaviour = task_file["end_behaviour"].as<float>();
        task.status = mw_msgs::Task::NOT_STARTED;
        task.type = (unsigned char) task_file["type"].as<int>();

        if (task.type == 0) {
          try {
            YAML::Node route_file = YAML::LoadFile(task_file["route"].as<std::string>());
            task.route.id = route_file["id"].as<unsigned int>();
            task.route.header.frame_id = "map";
            task.route.header.stamp = ros::Time::now();
            for (int j = 0; j < route_file["waypoints"].size(); j++) {
              geometry_msgs::PoseStamped pose;
              pose.header.frame_id = "map";
              pose.header.stamp = ros::Time::now();
              pose.pose.position.x = route_file["waypoints"][j][0].as<double>();
              pose.pose.position.y = route_file["waypoints"][j][1].as<double>();
              pose.pose.position.z = 0;
              task.route.waypoints.push_back(pose);
            }
          }
          catch (...) {
            ROS_ERROR("Error in %s", task_file["route"].as<std::string>().c_str());
            return 1;
          }
        }
        srv.request.duty.tasks.push_back(task);
      }
      catch (...) {
        ROS_ERROR("Error in %s", duty_file["task" + std::to_string(i + 1)].as<std::string>().c_str());
        return 1;
      }
    }

    if (client.call(srv)) {
      ROS_INFO("[set_route_client]: set route from '%s'", argv[1]);
    } else {
      ROS_ERROR("Failed to call service executive/set_route");
      return 1;
    }
  }
  catch (...) {
    ROS_ERROR("Error in %s", argv[1]);
    return 1;
  }

  return 0;

}
