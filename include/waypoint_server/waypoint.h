/**
 * @file waypoint.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief definition of the Waypoint Class
 * @version 0.1
 * @date 2019-06-26
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef WAYPOINT_SERVER_WAYPOINT_H_INCLUDED
#define WAYPOINT_SERVER_WAYPOINT_H_INCLUDED

#include <geometry_msgs/Pose.h>
#include <usv_navigation_msgs/Waypoint.h>

class Waypoint
{
public:
    Waypoint(uint8_t index,std::string mission,geometry_msgs::Pose pose,std::string frame_id,
        double lateral_torelance, double longitudinal_torelance, double yaw_torelance,
        std::vector<uint8_t> next_waypoint_index);
    ~Waypoint();
    const uint8_t index;
    const std::string mission;
    const geometry_msgs::Pose pose;
    const std::string frame_id;
    const double lateral_torelance;
    const double longitudinal_torelance;
    const double yaw_torelance;
    const std::vector<uint8_t> next_waypoint_index;
    usv_navigation_msgs::Waypoint toMsg();
};

#endif  //WAYPOINT_SERVER_WAYPOINT_H_INCLUDED