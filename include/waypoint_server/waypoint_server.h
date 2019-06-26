/**
 * @file waypoint_server.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief definition of the WaypointServer class
 * @version 0.1
 * @date 2019-06-26
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef WAYPOINT_SERVER_WAYPOINT_SERVER_H_INCLUDED
#define WAYPOINT_SERVER_WAYPOINT_SERVER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <rostate_machine/event_client.h>
#include <tf2_ros/transform_listener.h>

// Headers in this package
#include <waypoint_server/waypoint_parser.h>

/**
 * @brief definition of the WaypointServer class
 * 
 */
class WaypointServer
{
public:
    /**
    * @brief Constructor of WaypointServer class
    * 
    */
    WaypointServer(ros::NodeHandle nh,ros::NodeHandle pnh);
    /**
    * @brief destructor of WaypointServer class
    * 
    */
    ~WaypointServer();
private:
    /**
     * @brief node handler
     * 
     */
    ros::NodeHandle nh_;
    /**
     * @brief private node handler
     * 
     */
    ros::NodeHandle pnh_;
    /**
     * @brief event client for rostate_machine library
     * 
    */
    rostate_machine::EventClient event_client_;
    /**
    * @brief function for checking waypoint reached or not
    * 
    */
    boost::optional<rostate_machine::Event> checkWaypointReached();
    /**
     * @brief path of the waypoint.json file
     * 
     */
    std::string waypoint_json_path_;
    /**
     * @brief waypoint parser
     * 
     */
    WaypointParser waypoint_parser_;
    /**
     * @brief transform buffer
     * 
     */
    tf2_ros::Buffer tf_buffer_;
    /**
     * @brief transform listener
     * 
     */
    tf2_ros::TransformListener tf_listener_;
};

#endif  //WAYPOINT_SERVER_WAYPOINT_SERVER_H_INCLUDED