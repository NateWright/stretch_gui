#include "ObjectSegmenter.hpp"

ObjectSegmenter::ObjectSegmenter(ros::NodeHandlePtr nh) : nh_(nh) {
    clusterPub_ = nh_->advertise<sensor_msgs::PointCloud2>("/stretch_pc/cluster", 1000);
    pointPub_ = nh_->advertise<geometry_msgs::PointStamped>("/stretch_pc/centerPoint", 1000);
}

void ObjectSegmenter::segmentAndFind(const pcl::PointCloud<point>::Ptr& inputCloud, const point pointToFind) {
    pcl::PointCloud<point>::Ptr segmented_cloud(new pcl::PointCloud<point>);
    pcl::PointCloud<point>::Ptr background_filtered(new pcl::PointCloud<point>);
    pcl::PointCloud<point>::Ptr table_filtered_cloud(new pcl::PointCloud<point>);
    std::vector<pcl::PointIndices> clusters;

    background_filtered = filterDistance(inputCloud, 0.0, 1.0);
    if (background_filtered->size() == 0) {
        throw("segmentAndFind: background filtered cloud is empty");
    }
    table_filtered_cloud = filterTable(background_filtered);
    if (table_filtered_cloud->size() == 0) {
        throw("segmentAndFind: table filtered cloud is empty");
    }

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(table_filtered_cloud);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(10);
    reg.setPointColorThreshold(6);
    reg.setRegionColorThreshold(5);
    reg.setMinClusterSize(100);
    reg.extract(clusters);

    segmented_cloud = reg.getColoredCloud();
    segmented_cloud->header = inputCloud->header;

    ROS_INFO_STREAM("Picking event occurred");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster;

    ROS_INFO_STREAM("Looking for cluster");
    bool done = false;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(segmented_cloud);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    kdtree.nearestKSearch(pointToFind, 1, pointIdxNKNSearch, pointNKNSquaredDistance);

    int pos = pointIdxNKNSearch[0];

    for (pcl::PointIndices p : clusters) {
        cloud_cluster.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : p.indices) {
            cloud_cluster->push_back((*segmented_cloud)[idx]);
            if (idx == pos) {
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                done = true;
            }
        }
        if (done) {
            ROS_INFO_STREAM("Cluster found");
            break;
        }
    }

    float x = 0,
          y = 0,
          z = 0;
    int count = 0;
    for (const auto& p : cloud_cluster->points) {
        x += p.x;
        y += p.y;
        z += p.z;
        count++;
    }

    geometry_msgs::PointStamped pStamped;
    pStamped.point.x = x / count;
    pStamped.point.y = y / count;
    pStamped.point.z = z / count;
    pStamped.header.frame_id = inputCloud->header.frame_id;
    pointPub_.publish(pStamped);

    cloud_cluster->header.frame_id = inputCloud->header.frame_id;
    clusterPub_.publish(cloud_cluster);
}

pcl::PointCloud<point>::Ptr filterDistance(const pcl::PointCloud<point>::Ptr inputCloud, double fromDistance, double toDistance) {
    pcl::PointCloud<point>::Ptr segmented_cloud(new pcl::PointCloud<point>);
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(inputCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(fromDistance, toDistance);
    pass.filter(*indices);

    pcl::ExtractIndices<point> extract;
    extract.setInputCloud(inputCloud);
    extract.setIndices(indices);
    extract.filter(*segmented_cloud);
    return segmented_cloud;
}

pcl::PointCloud<point>::Ptr filterTable(const pcl::PointCloud<point>::Ptr inputCloud) {
    pcl::PointCloud<point>::Ptr segmented_cloud(new pcl::PointCloud<point>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<point> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.015);

    seg.setInputCloud(inputCloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<point> extract;
    extract.setInputCloud(inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*segmented_cloud);

    return segmented_cloud;
}