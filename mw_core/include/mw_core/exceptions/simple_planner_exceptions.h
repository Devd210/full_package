/**
 * 
 * @file simple_planner_exceptions.h
 * @brief The header file contains custom exceptions for the simple planner library.
 * @author Sourav Agrawal <sourav.agrawal@mowito.in>
 * 
 */

#pragma once

#include <mw_core/data_types/geometry_types.h>
#include <exception>

namespace mw_core
{
/*
* brief:The simple planner exception class. It has two points,start and end of the line and the distance to be increased.
*/
class simple_planner_exception : public std::exception
{
public:
    const char *what() const throw()
    {
        return "Simple planner exception has occurred";
    }
    Point point_a, point_b;
    double distance;
};
} // namespace mw_core