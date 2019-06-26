/**
 * @file waypoint_parser.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implimentation of the waypoint parser class
 * @version 0.1
 * @date 2019-06-26
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <waypoint_server/waypoint_parser.h>

WaypointParser::WaypointParser()
{

}

WaypointParser::~WaypointParser()
{

}

void WaypointParser::parse(std::string json_path)
{
    using namespace boost::property_tree;
    ptree pt;
    read_json(json_path, pt);
}