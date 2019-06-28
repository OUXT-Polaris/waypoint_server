/**
 * @file waypoint_parser.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implimentation of the waypoint parser class
 * @version 0.1
 * @date 2019-06-26
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <waypoint_server/waypoint_parser.h>

WaypointParser::WaypointParser()
{

}

WaypointParser::~WaypointParser()
{

}

std::map<uint8_t,Waypoint> WaypointParser::getWaypoints()
{
    std::map<uint8_t,Waypoint> ret;
    for(auto itr = waypoints_.begin(); itr != waypoints_.end(); itr++)
    {
        Waypoint wp = *itr;
        std::pair<uint8_t,Waypoint> pair = std::make_pair(wp.index,wp);
        ret.insert(pair);
    }
    return ret;
}

void WaypointParser::parse(std::string json_path)
{
    using namespace boost::property_tree;
    ptree pt;
    read_json(json_path, pt);
    boost::optional<uint8_t> start_waypoint_index = pt.get_optional<uint8_t>("start_waypoint_index");
    if(!start_waypoint_index)
    {
        ROS_ERROR_STREAM("failed to parse start_waypoint_index");
        std::exit(0);
    }
    start_waypoint_index_ = start_waypoint_index.get();
    BOOST_FOREACH (const ptree::value_type& child, pt.get_child("waypoints"))
    {
        const ptree& waypoint = child.second;
        boost::optional<uint8_t> index = waypoint.get_optional<uint8_t>("index");
        if(!index)
        {
            ROS_ERROR_STREAM("failed to parse waypoint index");
            std::exit(0);
        }
        boost::optional<std::string> mission = waypoint.get_optional<std::string>("mission");
        if(!mission)
        {
            ROS_ERROR_STREAM("failed to parse mission");
            std::exit(0);
        }
        boost::optional<geometry_msgs::Point> position = getPosition(waypoint);
        if(!position)
        {
            ROS_ERROR_STREAM("failed to parse position");
            std::exit(0);
        }
        boost::optional<geometry_msgs::Quaternion> orientation = getOrientation(waypoint);
        if(!orientation)
        {
            ROS_ERROR_STREAM("failed to parse orientation");
            std::exit(0);
        }
        boost::optional<double> lateral_tolerance = waypoint.get_optional<double>("tolerance.lateral");
        if(!lateral_tolerance)
        {
            ROS_ERROR_STREAM("failed to parse lateral tolerance.");
            std::exit(0);
        }
        boost::optional<double> longitudal_tolerance = waypoint.get_optional<double>("tolerance.longitudal");
        if(!longitudal_tolerance)
        {
            ROS_ERROR_STREAM("failed to parse longitudal tolerance.");
            std::exit(0);
        }
        boost::optional<double> yaw_tolerance = waypoint.get_optional<double>("tolerance.yaw");
        if(!yaw_tolerance)
        {
            ROS_ERROR_STREAM("failed to parse yaw tolerance.");
            std::exit(0);
        }
        boost::optional<std::string> frame_id = waypoint.get_optional<std::string>("frame_id");
        if(!frame_id)
        {
            ROS_ERROR_STREAM("failed to parse frame_id");
            std::exit(0);
        }
        boost::optional<std::vector<uint8_t> > next_waypoint_index = getNextWaypointIndex(waypoint);
        if(!next_waypoint_index)
        {
            ROS_ERROR_STREAM("failed to parse next waypoint index");
            std::exit(0);
        }
        geometry_msgs::Pose pose;
        pose.position = position.get();
        pose.orientation = orientation.get();
        Waypoint wp(index.get(),mission.get(),pose,frame_id.get(),
            lateral_tolerance.get(),longitudal_tolerance.get(),yaw_tolerance.get(),
            next_waypoint_index.get());
        waypoints_.push_back(wp);
    }
}

boost::optional<std::vector<uint8_t> > WaypointParser::getNextWaypointIndex(boost::property_tree::ptree tree)
{
    using namespace boost::property_tree;
    std::vector<uint8_t> ret;
    BOOST_FOREACH (const ptree::value_type& child, tree.get_child("next_waypoint_index"))
    {
        const ptree& p = child.second;
        boost::optional<uint8_t> x = p.get_optional<uint8_t>("");
        if(!x)
        {
            return boost::none;
        }
        ret.push_back(x.get());
    }
    return ret;
}

boost::optional<geometry_msgs::Quaternion> WaypointParser::getOrientation(boost::property_tree::ptree tree)
{
    boost::optional<double> x = tree.get_optional<double>("orientation.x");
    boost::optional<double> y = tree.get_optional<double>("orientation.y");
    boost::optional<double> z = tree.get_optional<double>("orientation.z");
    boost::optional<double> w = tree.get_optional<double>("orientation.w");
    if(x && y && z && w)
    {
        geometry_msgs::Quaternion q;
        q.x = x.get();
        q.y = y.get();
        q.z = z.get();
        q.w = w.get();
        return q;
    }
    else
    {
        return boost::none;
    }
}

boost::optional<geometry_msgs::Point> WaypointParser::getPosition(boost::property_tree::ptree tree)
{
    boost::optional<double> x = tree.get_optional<double>("position.x");
    boost::optional<double> y = tree.get_optional<double>("position.y");
    boost::optional<double> z = tree.get_optional<double>("position.z");
    if(x && y && z)
    {
        geometry_msgs::Point p;
        p.x = x.get();
        p.y = y.get();
        p.z = z.get();
        return p;
    }
    else
    {
        return boost::none;
    }
}