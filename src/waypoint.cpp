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

visualization_msgs::MarkerArray Waypoint::toMarkerMsg(std_msgs::ColorRGBA color)
{
    visualization_msgs::MarkerArray marker;
    visualization_msgs::Marker pose_marker;
    pose_marker.header.stamp = ros::Time::now();
    pose_marker.header.frame_id = frame_id;
    pose_marker.ns = "pose_marker";
    pose_marker.id = 0;
    pose_marker.type = pose_marker.CUBE;
    pose_marker.action = pose_marker.ADD;
    pose_marker.pose = pose;
    pose_marker.scale.x = longitudinal_torelance;
    pose_marker.scale.y = lateral_torelance;
    pose_marker.scale.z = 1.0;
    pose_marker.color = color;
    marker.markers.push_back(pose_marker);
    return marker;
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

bool Waypoint::reached(geometry_msgs::PoseStamped robot_pose,tf2_ros::Buffer tf_buffer)
{
    if(robot_pose.header.frame_id != frame_id)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer.lookupTransform(frame_id, robot_pose.header.frame_id ,ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return false;
        }
        tf2::doTransform(robot_pose,robot_pose,transform_stamped);
    }
    double target_yaw = quaternion_operation::convertQuaternionToEulerAngle(pose.orientation).z;
    double yaw = quaternion_operation::convertQuaternionToEulerAngle(robot_pose.pose.orientation).z;
    polygon poly;
    double x0 = pose.position.x + 0.5*lateral_torelance*std::cos(yaw) + 0.5*longitudinal_torelance*std::sin(0.5*M_PI-yaw);
    double y0 = pose.position.y + 0.5*lateral_torelance*std::sin(yaw) - 0.5*longitudinal_torelance*std::cos(0.5*M_PI-yaw);
    poly.outer().push_back(point(x0, y0));
    double x1 = pose.position.x + 0.5*lateral_torelance*std::cos(yaw) - 0.5*longitudinal_torelance*std::sin(0.5*M_PI-yaw);
    double y1 = pose.position.y + 0.5*lateral_torelance*std::sin(yaw) + 0.5*longitudinal_torelance*std::cos(0.5*M_PI-yaw);
    poly.outer().push_back(point(x1, y1));
    double x2 = pose.position.x - 0.5*lateral_torelance*std::cos(yaw) - 0.5*longitudinal_torelance*std::sin(0.5*M_PI-yaw);
    double y2 = pose.position.y - 0.5*lateral_torelance*std::sin(yaw) + 0.5*longitudinal_torelance*std::cos(0.5*M_PI-yaw);
    poly.outer().push_back(point(x2, y2));
    double x3 = pose.position.x - 0.5*lateral_torelance*std::cos(yaw) + 0.5*longitudinal_torelance*std::sin(0.5*M_PI-yaw);
    double y3 = pose.position.y - 0.5*lateral_torelance*std::sin(yaw) - 0.5*longitudinal_torelance*std::cos(0.5*M_PI-yaw);
    poly.outer().push_back(point(x3, y3));
    point pt(robot_pose.pose.position.x, robot_pose.pose.position.y);
    if(boost::geometry::within(pt, poly))
    {
        double diff_angle = getDiffAngle(yaw,target_yaw);
        if(diff_angle < yaw_torelance)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}