/**
 * @file waypoint_server_node.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief main function of the waypoint_server_node
 * @version 0.1
 * @date 2019-06-26
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// Headers in Ros
#include <ros/ros.h>

// Headers in this package
#include <waypoint_server/waypoint_server.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoint_server_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    WaypointServer server(nh,pnh);
    ros::spin();
    return 0;
}