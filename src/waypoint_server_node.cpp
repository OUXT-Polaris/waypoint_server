// Headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoint_server_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::spin();
    return 0;
}