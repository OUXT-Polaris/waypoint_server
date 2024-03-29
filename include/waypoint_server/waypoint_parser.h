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

// Headers in this package
#include <waypoint_server/waypoint.h>

// Headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

// Headers in STL
#include <map>

/**
 * @brief Waypoint parser
 * 
 */
class WaypointParser
{
public:
    WaypointParser();
    ~WaypointParser();
    /**
     * @brief parse json file
     * @param json_path filepath of the waypoint json file
     */
    void parse(std::string json_path);
    /**
     * @brief get waypoints
     * @return std::vector of the parsed waypoints
     */
    std::vector<Waypoint> getWaypoints(){return waypoints_;};
    /**
     * @brief get start waypoint index
     * @return start waypoint index
     */
    int16_t getStartWaypointIndex(){return start_waypoint_index_;};

    boost::optional<Waypoint> getStartWaypoint()
    {
        for(auto itr = waypoints_.begin(); itr != waypoints_.end(); itr++)
        {
            if(itr->index == start_waypoint_index_)
            {
                return *itr;
            }
        }
        return boost::none;
    }
private:
    /**
     * @brief parsed waypoints
     * 
     */
    std::vector<Waypoint> waypoints_;
    /**
     * @brief parse json tree and get geometry_msgs::Point
     * 
     */
    boost::optional<geometry_msgs::Point> getPosition(boost::property_tree::ptree tree);
    /**
     * @brief parse json tree and get geometry_msgs::Quaternion
     * 
     */
    boost::optional<geometry_msgs::Quaternion> getOrientation(boost::property_tree::ptree tree);
    /**
     * @brief parse json tree and get next waypoint index
     * 
     */
    boost::optional<std::vector<int16_t> > getNextWaypointIndex(boost::property_tree::ptree tree);
    /**
     * @brief start waypoint index
     * 
     */
    int16_t start_waypoint_index_;
};

#endif  //WAYPOINT_SERVER_WAYPOINT_PARSER_H_INCLUDED