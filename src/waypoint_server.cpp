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
    tf_buffer_ptr_(new tf2_ros::Buffer),
    tf_listener_(*tf_buffer_ptr_)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("waypoint_json_path", waypoint_json_path_, "");
    pnh_.param<std::string>("current_pose_topic", current_pose_topic_, "/current_pose");
    waypoint_parser_.parse(waypoint_json_path_);
    waypoints_ = waypoint_parser_.getWaypoints();
    current_waypoint_index_ = waypoint_parser_.getStartWaypointIndex();
    waypoint_pub_ = pnh_.advertise<usv_navigation_msgs::Waypoint>("target_waypoint",1);
    marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("target_waypoint/marker",1);
    navigation_event_client_.registerCallback(std::bind(&WaypointServer::checkWaypointReached, this),"WaypointServer::checkWaypointReached");
    navigation_event_client_.registerCallback(std::bind(&WaypointServer::loadNextWaypoint, this),"WaypointServer::loadNextWaypoint");
    mission_event_client_.run();
    navigation_event_client_.run();
    current_pose_sub_ = nh_.subscribe(current_pose_topic_,1,&WaypointServer::currentPoseCallback,this);
}

WaypointServer::~WaypointServer()
{

}

void WaypointServer::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    current_pose_ = *msg;
    return;
}

boost::optional<rostate_machine::Event> WaypointServer::checkWaypointReached()
{
    boost::optional<rostate_machine::State> current_state = mission_event_client_.getCurrentState();
    boost::optional<Waypoint> current_waypoint = getCurrentWaypoint();
    if(current_state && current_waypoint && current_pose_)
    {
        bool reached = current_waypoint->reached(*current_pose_,tf_buffer_ptr_);
        waypoint_pub_.publish(current_waypoint->toMsg());
        if(reached)
        {
            std_msgs::ColorRGBA red;
            red.r = 1.0;
            red.g = 0.0;
            red.b = 0.0;
            red.a = 0.3;
            marker_pub_.publish(current_waypoint->toMarkerMsg(red));
            rostate_machine::Event event;
            event.trigger_event_name = "reach_waypoint";
            return event;
        }
        std_msgs::ColorRGBA green;
        green.r = 0.0;
        green.g = 1.0;
        green.b = 0.0;
        green.a = 0.3;
        marker_pub_.publish(current_waypoint->toMarkerMsg(green));
    }
    return boost::none;
}

boost::optional<rostate_machine::Event> WaypointServer::loadNextWaypoint()
{
    boost::optional<rostate_machine::State> current_state = mission_event_client_.getCurrentState();
    boost::optional<Waypoint> current_waypoint = getCurrentWaypoint();
    if(current_state && current_waypoint && current_pose_)
    {
        bool reached = current_waypoint->reached(*current_pose_,tf_buffer_ptr_);
        if(reached)
        {
            /**
             * If reached the waypoint end
             * 
             */
            if(current_waypoint->isWaypointEnd())
            {
                rostate_machine::Event event;
                event.trigger_event_name = "reach_waypoint_end";
                return event;
            }
            std::vector<uint8_t> next_waypoint_index = current_waypoint->next_waypoint_index;
            for(auto itr = next_waypoint_index.begin(); itr != next_waypoint_index.end(); itr++)
            {
                boost::optional<Waypoint> next_waypoint_candidate = getTargetWaypoint(*itr);
                /**
                 * @brief If the next waypoint is found
                 * 
                 */
                if(next_waypoint_candidate && (next_waypoint_candidate->mission==current_waypoint->mission))
                {
                    current_waypoint_index_ = next_waypoint_candidate->index;
                    rostate_machine::Event event;
                    event.trigger_event_name = "load_waypoint";
                    return event;
                }
            }
        }
    }
    return boost::none;
}

boost::optional<Waypoint>  WaypointServer::getTargetWaypoint(int target_waypoint_index)
{
    for(auto itr = waypoints_.begin(); itr != waypoints_.end(); itr++)
    {
        if(itr->index == target_waypoint_index)
        {
            return *itr;
        }
    }
    return boost::none;
}

boost::optional<Waypoint> WaypointServer::getCurrentWaypoint()
{
    for(auto itr = waypoints_.begin(); itr != waypoints_.end(); itr++)
    {
        if(itr->index == current_waypoint_index_)
        {
            return *itr;
        }
    }
    return boost::none;
}