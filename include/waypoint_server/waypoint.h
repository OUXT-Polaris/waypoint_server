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

class Waypoint
{
public:
    Waypoint(int index,std::string mission,geometry_msgs::Pose pose,std::string frame_id,
        double lateral_tolerance, double longitudal_tolerance, double orientation_tolerance,
        std::vector<int> next_waypoint_index);
    ~Waypoint();
    const int index;
    const std::string mission;
    const geometry_msgs::Pose pose;
    const std::string frame_id;
    const double lateral_tolerance;
    const double longitudal_tolerance;
    const double orientation_tolerance;
    const std::vector<int> next_waypoint_index;
};

#endif  //WAYPOINT_SERVER_WAYPOINT_H_INCLUDED