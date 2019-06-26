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

Waypoint::Waypoint(int index,std::string mission,geometry_msgs::Pose pose,std::string frame_id,
    double lateral_tolerance, double longitudal_tolerance, double orientation_tolerance,
    std::vector<int> next_waypoint_index) 
        : index(index), mission(mission), pose(pose), frame_id(frame_id),
            lateral_tolerance(lateral_tolerance), longitudal_tolerance(longitudal_tolerance), orientation_tolerance(orientation_tolerance),
            next_waypoint_index(next_waypoint_index)
{
    
}

Waypoint::~Waypoint()
{

}