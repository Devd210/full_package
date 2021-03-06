/**
 * 
 * @file simple_planner.cpp
 * @brief This file implements the SimplePlanner class
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 *
 */

#include <global_planner/simple_planner.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(global_planner::SimplePlanner, global_planner::AbstractPlanner);

namespace global_planner
{

SimplePlanner::SimplePlanner()
{
}

SimplePlanner::~SimplePlanner()
{
}

void SimplePlanner::initialize(const std::string &name)
{
  AbstractPlanner::initialize(name);
  ros::NodeHandle private_nh("~/simple_planner");
  private_nh.param("radii", radii_, 0.5);
  private_nh.param("straight_point_distance", step_, 0.1);
  private_nh.param("corner_density", corner_density_, 0.1);

  dsrv_ = new dynamic_reconfigure::Server<global_planner::SimplePlannerConfig>(private_nh);
  dynamic_reconfigure::Server<global_planner::SimplePlannerConfig>::CallbackType
      cb = boost::bind(&SimplePlanner::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

int SimplePlanner::getRadius(double &radius, std::vector<double> &radii, Line line1, Line line2)
{
  radius = radii[0];
  return 0;
}

double SimplePlanner::calculateDistance(Point a, Point b)
{
  return std::sqrt(std::pow(a.y - b.y, 2) + std::pow(a.x - b.x, 2));
}

int SimplePlanner::getCenter(Point &center, Point a, Point b, double radius, double orthoAngle)
{
  double slopeAngle = 0.0;
  if (std::abs((orthoAngle * 180 / M_PI) - 0) < TOL_DEG)
  {
    slopeAngle = M_PI / 2;
  }
  else if (std::abs((orthoAngle * 180 / M_PI) - 180) < TOL_DEG)
  {
    slopeAngle = 3 * M_PI / 2;
  }
  else if (std::abs((orthoAngle * 90 / M_PI) - 0) < TOL_DEG)
  {
    slopeAngle = M_PI;
  }
  else
  {
    slopeAngle = std::atan(-1 / std::tan(orthoAngle));
  }

  double cx = a.x - radius * std::cos(slopeAngle);
  double cy = a.y - radius * std::sin(slopeAngle);

  if (std::abs(calculateDistance(Point(cx, cy), a) - calculateDistance(Point(cx, cy), b)) < TOL)
  {
    center = Point(cx, cy);
    return 1;
  }
  cx = a.x + radius * std::cos(slopeAngle);
  cy = a.y + radius * std::sin(slopeAngle);

  center = Point(cx, cy);
  return 0;
}

double SimplePlanner::checkRange(double angle)
{
  if (std::abs(angle) < TOL_RAD)
  {
    angle = 0;
  }
  while (angle < 0)
  {
    angle += 2 * M_PI;
  }
  while (angle > 2 * M_PI)
  {
    angle -= 2 * M_PI;
  }
  return angle;
}

double SimplePlanner::principleAngle(double angle)
{
  angle = checkRange(angle);
  if (angle > M_PI)
  {
    angle -= 2 * M_PI;
  }
  else if (angle < -M_PI)
  {
    angle += 2 * M_PI;
  }
  return angle;
}

double SimplePlanner::angleDifference(double angle1, double angle2)
{
  return principleAngle(checkRange(angle1) - checkRange(angle2));
}

int SimplePlanner::removeDuplicates()
{
  for (int i = 0; i < this->finalRoute.size() - 1; i++)
  {
    if (std::abs(this->finalRoute[i].x - this->finalRoute[i + 1].x) < TOL && std::abs(this->finalRoute[i].y - this->finalRoute[i + 1].y) < TOL)
    {
      this->finalRoute.erase(this->finalRoute.begin() + i + 1);
    }
  }
  return 0;
}

int SimplePlanner::getHeading()
{
  int i;
  for (i = 0; i < this->finalRoute.size() - 1; i++)
  {
    this->finalRoute[i].heading = std::atan2(this->finalRoute[i + 1].y - this->finalRoute[i].y,
                                             this->finalRoute[i + 1].x - this->finalRoute[i].x);
  }
  this->finalRoute[i].heading = this->finalRoute[i - 1].heading;
  return 0;
}

int SimplePlanner::getArc(std::vector<Point> &points,
                          Point center,
                          Point start,
                          Point end,
                          double radius,
                          double density)
{
  if (density - 0 < TOL)
  {
    std::__throw_invalid_argument("Corner density must be greater than zero");
  }
  Line lineCA = Line(center, start);
  lineCA.slopeAngle = checkRange(lineCA.slopeAngle);
  Line lineCB = Line(center, end);
  lineCB.slopeAngle = checkRange(lineCB.slopeAngle);
  int max_index = std::floor(M_PI / density);
  int total_index = std::floor(std::abs((lineCB.slopeAngle - lineCA.slopeAngle) / density));
  if (total_index > max_index)
  {
    total_index = 2 * max_index - total_index;
  }
  double start_angle = lineCA.slopeAngle;
  int sign = 1;
  if (std::abs(angleDifference(lineCB.slopeAngle, start_angle + (total_index - 1) * density)) >= 3 * density)
  {
    sign = -1;
  }
  for (int i = 0; i < total_index; i++)
  {
    points.push_back(Point(center.x + radius * std::cos(start_angle + sign * i * density),
                           center.y + radius * std::sin(start_angle + sign * i * density)));
  }
  return 0;
}

int SimplePlanner::getStraightPathPoints(std::vector<Point> &points, Point start, Point end, double step)
{
  if (step - 0 < TOL)
  {
    std::__throw_invalid_argument("Straight points distance must be greater than zero");
  }
  if (std::abs(start.x - end.x) < TOL && std::abs(start.y - end.y) < TOL)
  {
    points.push_back(end);
    return 0;
  }

  points.push_back(start);
  double slopeAngle = std::atan2(end.y - start.y, end.x - start.x);
  double distance = calculateDistance(start, end);
  double covered = step;
  while (covered < distance)
  {
    points.push_back(Point(start.x + std::cos(slopeAngle) * covered, start.y + std::sin(slopeAngle) * covered));
    if (distance - covered < step)
    {
      points.push_back(end);
      covered += step;
      break;
    }
    covered += step;
  }
  return 0;
}

int SimplePlanner::addPointsToRoute(std::vector<Line> lines,
                                    std::vector<double> radii,
                                    double cornerDensity,
                                    double step)
{
  std::vector<Point> points;
  this->finalRoute.push_back(Point(lines[0].start.x, lines[0].start.y));
  Point end_of_last_arc = lines[0].start;
  for (int i = 0; i < lines.size() - 1; i++)
  {
    //Checking if there is no turn on the waypoint
    double slopeAngle1 = lines[i].slopeAngle;
    double slopeAngle2 = lines[i + 1].slopeAngle;
    double angDifference = angleDifference(slopeAngle1, slopeAngle2);
    if (std::abs(angDifference) < TOL_RAD)
    {
      continue;
    }

    //Get radius of curvature needed for turn
    double radius;
    getRadius(radius, radii, lines[i], lines[i + 1]);
    if (radius - 0 < TOL)
    {
      std::__throw_invalid_argument("Set of radii can't be empty or zero");
    }
    Point corner = lines[i + 1].start;
    //The difference of the angle is equal to the angle made by the radii of the circle at the point of contact with tangent
    double tangentLength = std::abs(radius * std::tan(angDifference / 2));
    //Beginning of curve
    Point
        t1 = Point(corner.x - tangentLength * std::cos(slopeAngle1), corner.y - tangentLength * std::sin(slopeAngle1));
    //Ending of curve
    Point
        t2 = Point(corner.x + tangentLength * std::cos(slopeAngle2), corner.y + tangentLength * std::sin(slopeAngle2));
    //Get the center of the turn
    //Checking if the angle of turn is suitable for the given points
    double distance1 = std::abs(calculateDistance(lines[i + 1].start, lines[i + 1].end)) - tangentLength;
    double distance2 = std::abs(calculateDistance(end_of_last_arc, lines[i].end)) - tangentLength;
    if (distance1 < (-TOL))
    {
      //The distance between waypoints must be greater than the distance between corner and beginning/ending of the curve
      simple_planner_exception exp;
      exp.point_a = lines[i + 1].start;
      exp.point_b = lines[i + 1].end;
      exp.distance = std::abs(distance1);
      throw exp;
    }
    if (distance2 < (-TOL))
    {
      simple_planner_exception exp;
      exp.point_a = lines[i].start;
      exp.point_b = lines[i].end;
      exp.distance = std::abs(distance2);
      throw exp;
    }
    Point center;
    getCenter(center, t1, t2, radius, slopeAngle1);
    points.clear();
    getStraightPathPoints(points, this->finalRoute[this->finalRoute.size() - 1], t1, step);
    this->finalRoute.insert(this->finalRoute.end(), points.begin() + 1, points.end());
    this->finalRoute.push_back(t1);
    points.clear();
    getArc(points, center, t1, t2, radius, cornerDensity);
    this->finalRoute.insert(this->finalRoute.end(), points.begin() + 1, points.end());
    this->finalRoute.push_back(t2);
    end_of_last_arc = t2;
  }
  points.clear();
  getStraightPathPoints(points,
                        this->finalRoute[this->finalRoute.size() - 1],
                        Point(lines[lines.size() - 1].end.x, lines[lines.size() - 1].end.y),
                        step);
  this->finalRoute.insert(this->finalRoute.end(), points.begin() + 1, points.end());

  return 0;
}

int SimplePlanner::fillRoute(std::vector<Line> &lines, std::vector<Point> waypoints)
{
  for (int i = 0; i < waypoints.size() - 1; i++)
  {
    lines.push_back(Line(waypoints[i], waypoints[i + 1]));
  }
  return 0;
}

std::vector<Point> SimplePlanner::getFinalRoute()
{
  return this->finalRoute;
}

int SimplePlanner::generateRoute(std::vector<Point> waypoints,
                                 std::vector<double> radii,
                                 double cornerDensity,
                                 double step)
{
  finalRoute.clear();
  if (radii.size() == 0)
  {
    radii.push_back(0.1);
  }
  std::vector<Line> initialLines;
  int status;
  status = fillRoute(initialLines, waypoints);
  status = addPointsToRoute(initialLines, radii, cornerDensity, step);
  status = removeDuplicates();
  status = getHeading();
  return 0;
}

unsigned int SimplePlanner::makePlan(const std::vector<geometry_msgs::PoseStamped> &waypoints,
                                     double tolerance,
                                     std::vector<geometry_msgs::PoseStamped> &plan,
                                     double &cost,
                                     std::string &message)
{
  try
  {
    std::vector<Point> wp;
    for (auto &w : waypoints)
      wp.emplace_back(Point(w.pose.position.x, w.pose.position.y, 0.0));

    std::vector<double> radii;
    radii.push_back(radii_);

    if (generateRoute(wp, radii, corner_density_, step_) == 0)
    {
      message = "Plan was successfully computed";
      mw_core::convertPointToPose(finalRoute, plan);
      for (int i = 0; i < plan.size(); i++)
      {
        plan[i].header.frame_id = "map";
      }
      return 0;
    }
    else
    {
      message = "Planning has failed!";
      return 11;
    }
  }
  catch (simple_planner_exception &e)
  {
    message = e.what();
    char msg[250];
    sprintf(msg,
            "\nThe distance between waypoints (%.2lf,%.2lf) and (%.2lf,%.2lf) is not appropriate for the given radius of turn. Increase the distance atleast by %lf",
            e.point_a.x,
            e.point_a.y,
            e.point_b.x,
            e.point_b.y,
            e.distance);
    message.append(msg);
    return 12;
  }
  catch (std::invalid_argument &e)
  {
    message = e.what();
    return 13;
  }
  catch (std::exception &e)
  {
    message = e.what();
    return 14;
  }
}

} // namespace global_planner