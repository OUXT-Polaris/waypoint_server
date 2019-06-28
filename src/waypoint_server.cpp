/**
 * @file waypoint_server.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implimentation of the WaypointServer Class
 * @version 0.1
 * @date 2019-06-26
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// Headers in this package
#include <waypoint_server/waypoint_server.h>

WaypointServer::WaypointServer(ros::NodeHandle nh,ros::NodeHandle pnh)
    : mission_event_client_(nh,pnh,"mission_state_machine_node"), 
    navigation_event_client_(nh,pnh,"navigation_state_machine_node"),
    tf_listener_(tf_buffer_)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("waypoint_json_path", waypoint_json_path_, "");
    waypoint_parser_.parse(waypoint_json_path_);
    waypoints_ = waypoint_parser_.getWaypoints();
    mission_event_client_.run();
    navigation_event_client_.run();
}

WaypointServer::~WaypointServer()
{

}

boost::optional<rostate_machine::Event> WaypointServer::checkWaypointReached()
{

}