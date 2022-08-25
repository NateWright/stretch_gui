#include "ObjectSegmenter.hpp"

ObjectSegmenter::ObjectSegmenter(ros::NodeHandlePtr nh) : nh_(nh) {
    clusterPub_ = nh_->advertise<sensor_msgs::PointCloud2>("/stretch_pc/cluster", 1000);
    // testPub_ = nh_->advertise<sensor_msgs::PointCloud2>("/stretch_pc/test", 1000);
    pointPub_ = nh_->advertise<geometry_msgs::PointStamped>("/stretch_pc/centerPoint", 1000);
}

void ObjectSegmenter::segmentAndFind(const pcl::PointCloud<Point>::Ptr& inputCloud, const Point pointToFind, const tf2_ros::Buffer* buffer) {
    const std::string targetFrame = "base_link";
    const std::string outputFrame = inputCloud->header.frame_id;
    pcl::PointCloud<Point>::Ptr inputCloudTransformed(new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr segmented_cloud(new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr background_filtered(new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr table_filtered_cloud(new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr cluster(new pcl::PointCloud<Point>);
    std::vector<pcl::PointIndices> clusters;

    pcl_ros::transformPointCloud(targetFrame, *inputCloud, *inputCloudTransformed, *buffer);

    geometry_msgs::PointStamped point;
    point.header.frame_id = outputFrame;
    point.point = pclToGeo(pointToFind);

    point = buffer->transform(point, targetFrame);

    if (point.point.x * point.point.x + point.point.y * point.point.y > 1.00) {
        throw(ObjectOutOfRange());
    }

    const Point pointTransformed = geoToPcl(point.point);

    background_filtered = filterDistance(inputCloudTransformed, 0.0, 1.0, "x");
    if (background_filtered->size() == 0) {
        throw("segmentAndFind: background filtered cloud is empty");
    }

    background_filtered = filterDistance(background_filtered, 0.19, 1.0, "z");

    table_filtered_cloud = filterTable(background_filtered);
    if (table_filtered_cloud->size() == 0) {
        throw("segmentAndFind: table filtered cloud is empty");
    }

    pcl::search::Search<Point>::Ptr tree(new pcl::search::KdTree<Point>);

    pcl::RegionGrowingRGB<Point> reg;
    reg.setInputCloud(table_filtered_cloud);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(10);
    reg.setPointColorThreshold(6);
    reg.setRegionColorThreshold(5);
    reg.setMinClusterSize(100);
    reg.extract(clusters);

    segmented_cloud = reg.getColoredCloud();

    ROS_INFO_STREAM("Picking event occurred");

    pcl::PointCloud<Point>::Ptr cloud_cluster;

    ROS_INFO_STREAM("Looking for cluster");
    bool done = false;

    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(segmented_cloud);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    kdtree.nearestKSearch(pointTransformed, 1, pointIdxNKNSearch, pointNKNSquaredDistance);

    int pos = pointIdxNKNSearch[0];

    for (pcl::PointIndices p : clusters) {
        cloud_cluster.reset(new pcl::PointCloud<Point>);
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
    pStamped.header.frame_id = targetFrame;

    ROS_INFO_STREAM(distPlaneToPoint(planeA_, planeB_, planeC_, planeD_, pStamped.point.x, pStamped.point.y, pStamped.point.z));

    pointPub_.publish(buffer->transform(pStamped, outputFrame));

    cloud_cluster->header.frame_id = targetFrame;
    pcl_ros::transformPointCloud(outputFrame, *cloud_cluster, *cluster, *buffer);
    clusterPub_.publish(cluster);
}

geometry_msgs::Point pclToGeo(const Point p) {
    geometry_msgs::Point output;
    output.x = p.x;
    output.y = p.y;
    output.z = p.z;
    return output;
}

Point geoToPcl(const geometry_msgs::Point p) {
    Point output;
    output.x = p.x;
    output.y = p.y;
    output.z = p.z;
    return output;
}

pcl::PointCloud<Point>::Ptr filterDistance(const pcl::PointCloud<Point>::Ptr inputCloud, double fromDistance, double toDistance, std::string direction) {
    pcl::PointCloud<Point>::Ptr segmented_cloud(new pcl::PointCloud<Point>);
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(inputCloud);
    pass.setFilterFieldName(direction);
    pass.setFilterLimits(fromDistance, toDistance);
    pass.filter(*indices);

    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(inputCloud);
    extract.setIndices(indices);
    extract.filter(*segmented_cloud);
    return segmented_cloud;
}

pcl::PointCloud<Point>::Ptr ObjectSegmenter::filterTable(const pcl::PointCloud<Point>::Ptr inputCloud) {
    pcl::PointCloud<Point>::Ptr segmented_cloud(new pcl::PointCloud<Point>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<Point> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.015);

    seg.setInputCloud(inputCloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(inputCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*segmented_cloud);

    pcl::NormalEstimation<Point, pcl::Normal> ne;
    ne.setInputCloud(inputCloud);
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.03);
    ne.setIndices(inliers);
    ne.compute(*cloud_normals);

    float x = 0, y = 0, z = 0, count = 0;

    for (auto norm : *cloud_normals) {
        x += norm.normal_x;
        y += norm.normal_y;
        z += norm.normal_z;
        count++;
    }

    planeA_ = x / count;
    planeB_ = y / count;
    planeC_ = z / count;

    Point p = (*inputCloud)[inliers->indices.front()];

    planeD_ = -planeA_ * p.x - planeB_ * p.y - planeC_ * p.z;
    ROS_INFO_STREAM(planeA_);
    ROS_INFO_STREAM(planeB_);
    ROS_INFO_STREAM(planeC_);
    ROS_INFO_STREAM(planeD_);

    return segmented_cloud;
}

float distPlaneToPoint(float planeA, float planeB, float planeC, float planeD, float x, float y, float z) {
    return abs(planeA * x + planeB * y + planeC * z + planeD) / sqrt(planeA * planeA + planeB * planeB + planeC * planeC);
}