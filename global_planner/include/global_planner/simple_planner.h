/**
 *
 * @file simple_planner.h
 * @brief Header file for simple planner class and related structures
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 *
 */

#pragma once

#include <vector>
#include <ros/ros.h>
#include <mw_core/data_types/geometry_types.h>
#include <mw_core/exceptions/simple_planner_exceptions.h>
#include <mw_core/utility_functions.h>
#include <global_planner/base_planner.h>
#include <global_planner/SimplePlannerConfig.h>
#include <dynamic_reconfigure/server.h>

using mw_core::Line;
using mw_core::Point;
using mw_core::simple_planner_exception;

namespace global_planner
{
/**
 * @brief The simple planner class. This class can be used to generate simple straight line path between waypoints. Inherits the 
 *        AbstractPlanner class so that it can be used in the navigation stack as a plugin.
 */
class SimplePlanner : public AbstractPlanner
{
  /**
    * @section Private Variables
    */
  double const TOL = 0.001;                //Tolerance in meteres
  double const TOL_DEG = 0.1;              //Tolerance in Degrees
  double const TOL_RAD = 0.1 * M_PI / 180; //Tolerance in Radians
  std::vector<Point> finalRoute;           //Stores the final generated route for the specified waypoints

  double radii_;
  double corner_density_;
  double step_;

  dynamic_reconfigure::Server<global_planner::SimplePlannerConfig> *dsrv_;

  /**
    * @section Private functions
    */

  void reconfigureCB(global_planner::SimplePlannerConfig &config, uint32_t level)
  {
    radii_ = config.radii;
    corner_density_ = config.corner_density;
    step_ = config.straight_point_distance;
  }

  /**
    * @brief Chooses the best radius from a given set of radii
    * @param radius The choosen radius will be stored in this variable
    * @param radii A std::vector containing the suitable radii
    * @param line1 The first line
    * @param line2 The second line
    * @return indicates status of execution
    */
  int getRadius(double &radius, std::vector<double> &radii, Line line1, Line line2);
  /**
    * @brief Calculates the euclidean distance betweeen two points
    * @param a:The first point
    * @param b:The second point
    * @return the distance between the points
    */
  double calculateDistance(Point a, Point b);
  /**
    * @brief Determines the center of the circle which contains the arc
    * @param center The calculated center will be stored in this variable
    * @param a The first tangential point
    * @param b The second tangential point
    * @param radius The radius of the circle
    * @param orthoAngle The orthogonal angle of the first tangent
    * @return the status of execution
    */
  int getCenter(Point &center, Point a, Point b, double radius, double orthoAngle);
  /**
    * @brief Makes the angle between 0 to 2pi
    * @param angle The angle to be converted
    * @return the modified angle
    */
  double checkRange(double angle);
  /**
    * @brief Makes the angle between -pi to pi
    * @param angle The angle to be converted
    * @return the principle angle
    */
  double principleAngle(double angle);
  /**
    * @brief Determines the difference between two angles
    * @param angle1 The first angle
    * @param angle2 The second angle
    * @return the difference between the angles
    */
  double angleDifference(double angle1, double angle2);
  /**
    * @brief Removes the duplicate points in the final route
    * @return the status of execution
    */
  int removeDuplicates();
  /**
    * @brief Adds headings to the calculated route points
    * @return the status of execution
    */
  int getHeading();
  /**
    * @brief Generates the points in the arc
    * @param points The generated points will be stored in this variable
    * @param center The center of the circle
    * @param start The first tangential point
    * @param end The second tangential point
    * @param radius The radius of the circle
    * @param density The density of the points in the arc
    * @return the status of execution
    */
  int getArc(std::vector<Point> &points, Point center, Point start, Point end, double radius, double density);
  /**
    * @brief Generates the points in the straight line path
    * @param point The generated points will be stored in this variable
    * @param center The center of the circle
    * @param start The first tangential point
    * @param end The second tangential point
    * @param step The distance between two successive points in the straight path
    * @return value indicates status of execution
    */
  int getStraightPathPoints(std::vector<Point> &points, Point start, Point end, double step);
  /**
    * @brief Generates the route points
    * @param lines The lines connecting the waypoints
    * @param radii The set of suitable radius
    * @param cornerDensity The density of points in the corner
    * @param step The distance between two successive points in the straight path
    * @return value indicates status of execution
    */
  int addPointsToRoute(std::vector<Line> lines, std::vector<double> radii, double cornerDensity, double step);
  /**
    * @brief Generates the basic route from the given waypoints
    * @param lines The lines connecting the waypoints
    * @param waypoints The generated route will be stored in this variable
    * @return value indicates status of execution
    */
  int fillRoute(std::vector<Line> &lines, std::vector<Point> waypoints);
  /**
    * @brief This is inherited from the AbstractPlanner class, for documentation read base class documentation
    * @return value indicates status of execution
    */
  unsigned int makePlan(const std::vector<geometry_msgs::PoseStamped> &waypoints,
                        double tolerance,
                        std::vector<geometry_msgs::PoseStamped> &plan,
                        double &cost,
                        std::string &message);

public:
  /**
    * @section Public Functions
    */
  /**
    * @brief Returns the final route
    * @returns the points along the calculated route
    */
  std::vector<Point> getFinalRoute();
  /**
    * @brief Generates the route
    * @param waypoints The waypoints
    * @param radii The set of suitable radius
    * @param cornerDensity The density of points in the corner
    * @param step The distance between two successive points in the straight path
    * @return status of execution
    */
  int generateRoute(std::vector<Point> waypoints, std::vector<double> radii, double cornerDensity, double step);

  void initialize(const std::string &name);

  SimplePlanner();
  ~SimplePlanner();
};
} // namespace global_planner