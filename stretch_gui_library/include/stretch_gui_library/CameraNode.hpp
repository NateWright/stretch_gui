#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/common/distances.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

#include "ObjectSegmenter.hpp"

class CameraNode {
   public:
    explicit CameraNode(ros::NodeHandlePtr nh);
    ~CameraNode();

   private:
    ros::NodeHandlePtr nh_;

    ros::Subscriber colorCameraSub_;
    ros::Subscriber segmentedCameraSub_;
    ros::Subscriber centerPointSub_;

    ros::Publisher pointPick_;
    ros::Publisher cameraPub_;
    ros::Publisher cameraPointPub_;
    ros::Publisher clickInitiated_;

    ros::AsyncSpinner *spinner_;
    tf2_ros::Buffer *tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sceneClickCloud_;

    sensor_msgs::Image::Ptr img_;

    ObjectSegmenterPtr segmenter_;

    void cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &);
    void segmentedCameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &);

   signals:

    // Sends QImage with selected object on screen
    void imgUpdateWithObject(QImage);
    void distanceToTable(float);
    void clickSuccess();
    void clickFailure();
    void clickInitiated();
    void validPoint();
    void invalidPoint();
   public slots:
    void sceneClicked(QPoint press, QPoint release, QSize screen);
};
