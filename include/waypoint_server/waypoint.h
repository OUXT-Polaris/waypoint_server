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
    /**
     * @brief constructor of the Waypoint class
     * 
     */
    Waypoint(uint8_t index,std::string mission,geometry_msgs::Pose pose,std::string frame_id,
        double lateral_torelance, double longitudinal_torelance, double yaw_torelance,
        std::vector<uint8_t> next_waypoint_index);
    /**
     * @brief destructor of the Waypoint class
     * 
     */
    ~Waypoint();
    /**
     * @brief waypoint index
     * 
     */
    const uint8_t index;
    /**
     * @brief mission of the waypoint
     * 
     */
    const std::string mission;
    /**
     * @brief target pose of the waypoint
     * 
     */
    const geometry_msgs::Pose pose;
    /**
     * @brief frame_id of the pose
     * 
     */
    const std::string frame_id;
    /**
     * @brief lateral torelance of the target pose
     * 
     */
    const double lateral_torelance;
    /**
     * @brief longitudinal torelance of the target pose
     * 
     */
    const double longitudinal_torelance;
    /**
     * @brief yaw torelance of the target pose
     * 
     */
    const double yaw_torelance;
    /**
     * @brief next waypoint index
     * 
     */
    const std::vector<uint8_t> next_waypoint_index;
    /**
     * @brief Convert to ROS Message
     * 
     */
    usv_navigation_msgs::Waypoint toMsg();
};

#endif  //WAYPOINT_SERVER_WAYPOINT_H_INCLUDED