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

// Headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <usv_navigation_msgs/Waypoint.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <quaternion_operation/quaternion_operation.h>

// Headers in STL
#include <memory>

// Headers in Boost
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>

using point = boost::geometry::model::d2::point_xy<double>;
using polygon = boost::geometry::model::polygon<point>;
using box = boost::geometry::model::box<point>;

/**
 * @brief Definition of the waypoint
 * 
 */
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
     * @brief Convert to ROS message
     * 
     */
    usv_navigation_msgs::Waypoint toMsg();
    /**
     * @brief Convert to ROS marker message
     * 
     */
    visualization_msgs::MarkerArray toMarkerMsg(std_msgs::ColorRGBA color);
    /**
     * @brief check the pose was reached target waypoint or not 
     * @retval true pose is reached the waypoint
     * @retval false pose is not reached the waypoint
     */
    bool reached(geometry_msgs::PoseStamped pose,std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr);
    /**
     * @brief Check the waypoint is end or not
     * @retval true the waypoint is the waypoint end
     * @retval false the waypoint is not the waypoint end
     */
    bool isWaypointEnd()
    {
        if(next_waypoint_index.size() == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    };
private:
    /**
     * @brief calculate angle difference between two angles
     * 
     */
    double getDiffAngle(double angle0,double angle1)
    {
        double ret;
        double a0 = std::cos(angle0);
        double a1 = std::sin(angle0);
        double b0 = std::cos(angle1);
        double b1 = std::sin(angle1);
        ret = std::acos(a0*b0 + a1*b1);
        return ret;
    }
    double diff_angle_;
};

#endif  //WAYPOINT_SERVER_WAYPOINT_H_INCLUDED