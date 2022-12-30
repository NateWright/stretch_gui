#include "CameraNode.hpp"

CameraNode::CameraNode(ros::NodeHandlePtr nh) : nh_(nh) {
    segmenter_.reset(new ObjectSegmenter(nh_));
    std::string pointCloudTopic;
    nh->getParam("/stretch_gui/pointCloudTopic", pointCloudTopic);
    colorCameraSub_ = nh_->subscribe(pointCloudTopic, 30, &CameraNode::cameraCallback, this);
    segmentedCameraSub_ = nh_->subscribe("/stretch_pc/cluster", 30, &CameraNode::segmentedCameraCallback, this);
    sceneClickedSub_ = nh_->subscribe("/stretch_gui/scene_clicked", 30, &CameraNode::sceneClicked, this);
    pointPick_ = nh->advertise<geometry_msgs::PointStamped>("/clicked_point", 30);
    cameraPub_ = nh_->advertise<sensor_msgs::Image>("/stretch_gui/image", 30);
    cameraPointPub_ = nh_->advertise<sensor_msgs::Image>("/stretch_gui/image_selection", 30);
    clickInitiated_ = nh_->advertise<std_msgs::Empty>("/stretch_gui/click_initiated", 30);
    clickStatus_ = nh_->advertise<stretch_gui_library::PointStatus>("/stretch_gui/click_status", 20);

    tfBuffer_ = new tf2_ros::Buffer();
    tfListener_ = new tf2_ros::TransformListener(*tfBuffer_);
}
CameraNode::~CameraNode() {
    spinner_->stop();
    delete spinner_;
    delete tfListener_;
    delete tfBuffer_;
}

void CameraNode::cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc) {
    cloud_ = pc;

    const int width = pc->width, height = pc->height;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZRGB>(height, width, pcl::PointXYZRGB(0, 0, 0)));

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            rotatedCloud->at(height - 1 - y, x) = pc->at(x, y);
        }
    }

    pcl::toROSMsg(*rotatedCloud, *img_);
    cameraPub_.publish(*img_);
}

void CameraNode::segmentedCameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pc) {
    const int width = sceneClickCloud_->width, height = sceneClickCloud_->height;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(sceneClickCloud_);
    const int count = 20;
    std::vector<int> pointIdxNKNSearch(count);
    std::vector<float> pointNKNSquaredDistance(count);

    cv_bridge::CvImagePtr img;
    img = cv_bridge::toCvCopy(img_, sensor_msgs::image_encodings::BGR8);
    for (pcl::PointXYZRGB p : *pc) {
        kdtree.nearestKSearch(p, count, pointIdxNKNSearch, pointNKNSquaredDistance);
        for (int pos : pointIdxNKNSearch) {
            img->image.at<cv::Scalar>(height - 1 - pos / width, pos % width) = CV_RGB(255, 0, 0);
        }
    }
    cameraPointPub_.publish(img);
}

void CameraNode::sceneClicked(stretch_gui_library::PointClicked msg) {
    sceneClickCloud_ = cloud_;
    if (!sceneClickCloud_ || sceneClickCloud_->size() == 0) {
        stretch_gui_library::PointStatus outputMsg;
        outputMsg.success = false;
        outputMsg.msg = "Please try again";
        clickStatus_.publish(outputMsg);
        return;
    }
    uint32_t locX = msg.x * static_cast<double>(sceneClickCloud_->height) / static_cast<double>(msg.width);
    uint32_t locY = msg.y * static_cast<double>(sceneClickCloud_->width) / static_cast<double>(msg.height);

    try {
        if (locY > sceneClickCloud_->width || locX > sceneClickCloud_->height) {
            throw(std::runtime_error("Not in range"));
        }
        std_msgs::Empty b;
        clickInitiated_.publish(b);
        pcl::PointXYZRGB p = sceneClickCloud_->at(locY, sceneClickCloud_->height - locX);

        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
            throw(std::runtime_error("Point contains NaN"));
        }

        geometry_msgs::PointStamped::Ptr point(new geometry_msgs::PointStamped());
        point->header.frame_id = sceneClickCloud_->header.frame_id;

        point->point.x = p.x;
        point->point.y = p.y;
        point->point.z = p.z;

        try {
            // TODO
            // emit distanceToTable(segmenter_->segmentAndFind(sceneClickCloud_, p, tfBuffer_));
        } catch (ObjectOutOfRange& error) {
            ROS_INFO_STREAM("object out of range");
            stretch_gui_library::PointStatus outputMsg;
            outputMsg.success = false;
            outputMsg.msg = "Object out of Range";
            clickStatus_.publish(outputMsg);
            return;
        } catch (...) {
            ROS_INFO_STREAM("catch all");
            ROS_INFO_STREAM("Point cloud was empty after segmentation");
            stretch_gui_library::PointStatus outputMsg;
            outputMsg.success = false;
            outputMsg.msg = "No object in Scene";
            clickStatus_.publish(outputMsg);
            return;
        }
        pointPick_.publish(point);
        std_msgs::Bool outputMsg;
        outputMsg.data = true;
        clickStatus_.publish(outputMsg);
    } catch (...) {
        stretch_gui_library::PointStatus outputMsg;
        outputMsg.success = false;
        outputMsg.msg = "Segmentation failed";
        clickStatus_.publish(outputMsg);
        return;
    }
}
