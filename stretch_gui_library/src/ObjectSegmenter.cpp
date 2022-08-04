#include "ObjectSegmenter.hpp"

ObjectSegmenter::ObjectSegmenter(ros::NodeHandlePtr nh) : nh_(nh) {
    clusterPub_ = nh_->advertise<sensor_msgs::PointCloud2>("/stretch_pc/cluster", 1000);
    pointPub_ = nh_->advertise<geometry_msgs::PointStamped>("/stretch_pc/centerPoint", 1000);
    testPub_ = nh_->advertise<sensor_msgs::PointCloud2>("/stretch_gui/testCloud", 100);
}

// void ObjectSegmenter::segmentAndFind(const pcl::PointCloud<point>::Ptr& inputCloud, const point pointToFind) {
//     pcl::PointCloud<point>::Ptr vox_filtered_cloud(new pcl::PointCloud<point>);
//     pcl::PointCloud<point>::Ptr segmented_cloud(new pcl::PointCloud<point>);
//     std::vector<pcl::PointIndices> clusters;

//     // Down sample the point cloud
//     pcl::VoxelGrid<point> voxelFilter;
//     voxelFilter.setInputCloud(inputCloud);
//     voxelFilter.setLeafSize(0.015f, 0.015f, 0.015f);
//     voxelFilter.filter(*vox_filtered_cloud);

//     pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
//     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//     pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
//     normal_estimator.setSearchMethod(tree);
//     normal_estimator.setInputCloud(vox_filtered_cloud);
//     normal_estimator.setKSearch(50);
//     normal_estimator.compute(*normals);

//     pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
//     reg.setMinClusterSize(50);
//     reg.setMaxClusterSize(1000000);
//     reg.setSearchMethod(tree);
//     reg.setNumberOfNeighbours(30);
//     reg.setInputCloud(vox_filtered_cloud);
//     reg.setInputNormals(normals);
//     reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
//     reg.setCurvatureThreshold(1.0);

//     reg.extract(clusters);
//     segmented_cloud = reg.getColoredCloud();
//     segmented_cloud->header = inputCloud->header;
//     testPub_.publish(segmented_cloud);

//     ROS_INFO_STREAM("Picking event occurred");

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster;

//     ROS_INFO_STREAM("Looking for cluster");
//     bool done = false;

//     pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
//     kdtree.setInputCloud(segmented_cloud);
//     std::vector<int> pointIdxNKNSearch(1);
//     std::vector<float> pointNKNSquaredDistance(1);

//     kdtree.nearestKSearch(pointToFind, 1, pointIdxNKNSearch, pointNKNSquaredDistance);

//     int pos = pointIdxNKNSearch[0];

//     for (pcl::PointIndices p : clusters) {
//         cloud_cluster.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
//         for (const auto& idx : p.indices) {
//             cloud_cluster->push_back((*segmented_cloud)[idx]);
//             if (idx == pos) {
//                 cloud_cluster->width = cloud_cluster->size();
//                 cloud_cluster->height = 1;
//                 cloud_cluster->is_dense = true;
//                 done = true;
//             }
//         }
//         if (done) {
//             ROS_INFO_STREAM("Cluster found");
//             break;
//         }
//     }

//     float x = 0,
//           y = 0,
//           z = 0;
//     int count = 0;
//     for (const auto& p : cloud_cluster->points) {
//         x += p.x;
//         y += p.y;
//         z += p.z;
//         count++;
//     }

//     geometry_msgs::PointStamped pStamped;
//     pStamped.point.x = x / count;
//     pStamped.point.y = y / count;
//     pStamped.point.z = z / count;
//     pStamped.header.frame_id = inputCloud->header.frame_id;
//     pointPub_.publish(pStamped);

//     cloud_cluster->header.frame_id = inputCloud->header.frame_id;
//     clusterPub_.publish(cloud_cluster);
// }

void ObjectSegmenter::segmentAndFind(const pcl::PointCloud<point>::Ptr& inputCloud, const point pointToFind) {
    pcl::PointCloud<point>::Ptr vox_filtered_cloud(new pcl::PointCloud<point>);
    pcl::PointCloud<point>::Ptr segmented_cloud(new pcl::PointCloud<point>);
    std::vector<pcl::PointIndices> clusters;

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*inputCloud, *indices);

    // Down sample the point cloud
    pcl::VoxelGrid<point> voxelFilter;
    voxelFilter.setInputCloud(inputCloud);
    // voxelFilter.setIndices(indices);
    voxelFilter.setLeafSize(0.0155f, 0.0155f, 0.0155f);
    voxelFilter.filter(*vox_filtered_cloud);

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(vox_filtered_cloud);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(10);
    reg.setPointColorThreshold(6);
    reg.setRegionColorThreshold(5);
    reg.setMinClusterSize(10);

    reg.extract(clusters);

    segmented_cloud = reg.getColoredCloud();
    segmented_cloud->header = inputCloud->header;
    testPub_.publish(segmented_cloud);

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