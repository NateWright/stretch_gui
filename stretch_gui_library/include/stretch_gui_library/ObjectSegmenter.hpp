#include <pcl/common/distances.h>
// #include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>  // for pcl::removeNaNFromPointCloud
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <thread>
#include <vector>

#pragma once

using namespace std::chrono_literals;

typedef pcl::PointXYZRGB point;

class ObjectSegmenter {
   private:
    // Setup
    ros::NodeHandlePtr nh_;

    // Publishers and Subscribers
    ros::Publisher clusterPub_;
    ros::Publisher pointPub_;

   public:
    explicit ObjectSegmenter(ros::NodeHandlePtr nh);
    void segmentAndFind(const pcl::PointCloud<point>::Ptr&, const point);
};

pcl::PointCloud<point>::Ptr filterDistance(const pcl::PointCloud<point>::Ptr, double fromDistance, double toDistance);
pcl::PointCloud<point>::Ptr filterTable(const pcl::PointCloud<point>::Ptr);

typedef std::shared_ptr<ObjectSegmenter> ObjectSegmenterPtr;
