/**
 * @file waypoint.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implimentation of the Waypoint Class
 * @version 0.1
 * @date 2019-06-26
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <waypoint_server/waypoint.h>

Waypoint::Waypoint(uint8_t index,std::string mission,geometry_msgs::Pose pose,std::string frame_id,
    double lateral_torelance, double longitudinal_torelance, double yaw_torelance,
    std::vector<uint8_t> next_waypoint_index) 
        : index(index), mission(mission), pose(pose), frame_id(frame_id),
            lateral_torelance(lateral_torelance), longitudinal_torelance(longitudinal_torelance), yaw_torelance(yaw_torelance),
            next_waypoint_index(next_waypoint_index)
{
    
}

Waypoint::~Waypoint()
{

}

usv_navigation_msgs::Waypoint Waypoint::toMsg()
{
    usv_navigation_msgs::Waypoint msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();
    msg.pose = pose;
    msg.index = index;
    msg.next_waypoint_index = next_waypoint_index;
    msg.longitudinal_torelance = longitudinal_torelance;
    msg.lateral_torelance = lateral_torelance;
    msg.yaw_torelance = yaw_torelance;
    return msg;
}