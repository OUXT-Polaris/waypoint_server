/**
 * @file waypoint_parser.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief definition of the waypoint parser class
 * @version 0.1
 * @date 2019-06-26
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef WAYPOINT_SERVER_WAYPOINT_PARSER_H_INCLUDED
#define WAYPOINT_SERVER_WAYPOINT_PARSER_H_INCLUDED

// Headers in Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

class WaypointParser
{
public:
    WaypointParser();
    ~WaypointParser();
    void parse(std::string json_path);
};

#endif  //WAYPOINT_SERVER_WAYPOINT_PARSER_H_INCLUDED