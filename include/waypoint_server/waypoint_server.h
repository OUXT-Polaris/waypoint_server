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
#include <geometry_msgs/PoseStamped.h>

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
     * @brief Query current waypoint
     * 
     */
    boost::optional<Waypoint>  getCurrentWaypoint();
    /**
     * @brief Query target waypoint
     * 
     */
    boost::optional<Waypoint>  getTargetWaypoint(int target_waypoint_index);
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
     * @brief mission event client for rostate_machine library
     * 
    */
    rostate_machine::EventClient mission_event_client_;
    /**
     * @brief navigation event client for rostate_machine library
     * 
    */
    rostate_machine::EventClient navigation_event_client_;
    /**
    * @brief function for checking waypoint reached or not
    * 
    */
    boost::optional<rostate_machine::Event> checkWaypointReached();
    /**
     * @brief function for loading next waypoint
     * 
     */
    boost::optional<rostate_machine::Event> loadNextWaypoint();
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
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
    /**
     * @brief transform listener
     * 
     */
    tf2_ros::TransformListener tf_listener_;
    /**
     * @brief current waypoint index
     * 
     */
    int16_t current_waypoint_index_;
    /**
     * @brief parsed waypoint data
     * 
     */
    std::vector<Waypoint> waypoints_;
    /**
     * @brief ROS callback function for /current_pose topic
     * 
     */
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
    /**
     * @brief parameter for /current_pose topic name
     * 
     */
    std::string current_pose_topic_;
    /**
     * @brief ROS Subscriber for current_pose topic
     * 
     */
    ros::Subscriber current_pose_sub_;
    /**
     * @brief current pose of the robot.
     * 
     */
    boost::optional<geometry_msgs::PoseStamped> current_pose_;
    /**
     * @brief ROS marker publisher
     * 
     */
    ros::Publisher marker_pub_;
    /**
     * @brief ROS waypoint publisher
     * 
     */
    ros::Publisher waypoint_pub_;
};

#endif  //WAYPOINT_SERVER_WAYPOINT_SERVER_H_INCLUDED