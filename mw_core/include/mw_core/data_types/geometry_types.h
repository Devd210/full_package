/**
 * 
 * @file geometry_types.h
 * @brief The header file contains various types of geometry data types used throughout the stack.
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 *
 */
#pragma once
#include <cmath>

namespace mw_core
{
/*
* brief:The 2-D point data structure. A point has a x-coordinate, a y-coordinate and a heading.
*/
struct Point
{
    double x, y, heading;
    Point(double x, double y, double heading = 0.0)
    {
        this->x = x;
        this->y = y;
        this->heading = heading;
    }
    Point(Point *pt)
    {
        this->x = pt->x;
        this->y = pt->y;
        this->heading = pt->heading;
    }
    Point() {}
};

/*
* brief:The Line data structure. A line is represented by a starting 2-D point, an ending 2-D point and the slope of the line.
*/
struct Line
{
    //Data Structure for a Line
    Point start, end;
    double slopeAngle;
    Line(Point start, Point end)
    {
        this->start = start;
        this->end = end;
        this->slopeAngle = std::atan2((end.y - start.y), (end.x - start.x));
    }
};

} // namespace mw_core