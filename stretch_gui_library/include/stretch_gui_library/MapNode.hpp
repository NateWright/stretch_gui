#pragma once
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <stretch_gui_library/MapPose.h>
#include <stretch_gui_library/MoveCommand.h>
#include <stretch_gui_library/Point.h>
#include <stretch_gui_library/SetMapping.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>

using std::vector;
using stretch_gui_library::MapPose;
using stretch_gui_library::Point;

struct Size {
    uint32_t width;
    uint32_t height;
};

class MapNode {
   public:
    explicit MapNode(ros::NodeHandlePtr nodeHandle);
    ~MapNode();

   private:
    ros::NodeHandlePtr nh_;
    ros::Subscriber mapSub_;
    ros::Subscriber mapPointCloudSub_;
    ros::Publisher movePub_;
    ros::Publisher mapPub_;
    ros::Timer posTimer_;

    ros::AsyncSpinner* spinner_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener* tfListener_;

    Size mapSize_;
    Point origin_;
    double resolution_;

    geometry_msgs::PoseStamped robotHome_;
    nav_msgs::OccupancyGrid::ConstPtr mapMsg_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void posCallback(const ros::TimerEvent&);
    void mapPointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

    ros::Publisher hasHome_;
    void hasHome(bool b) {
        std_msgs::Bool msg;
        msg.data = b;
        hasHome_.publish(msg);
    }
    ros::Publisher robotPose_;

    ros::Subscriber moveRobotSub_;
    void moveRobotCallback(stretch_gui_library::MoveCommand msg);
    ros::ServiceServer setMapping_;
    bool setMapping(stretch_gui_library::SetMapping::Request& req, stretch_gui_library::SetMapping::Response& res);
    ros::ServiceServer setHome_;
    bool setHome(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    ros::Subscriber setHomeIfNone_;
    void setHomeIfNone(std_msgs::Empty msg);
    ros::ServiceServer navigateHome_;
    bool navigateHome(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
};

Point translateScreenToMap(Point p, Size screen, Size map);