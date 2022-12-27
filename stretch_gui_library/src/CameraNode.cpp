#include "CameraNode.hpp"

CameraNode::CameraNode(ros::NodeHandlePtr nh) : nh_(nh) {
    segmenter_.reset(new ObjectSegmenter(nh_));
    std::string pointCloudTopic;
    nh->getParam("/stretch_gui/pointCloudTopic", pointCloudTopic);
    colorCameraSub_ = nh_->subscribe(pointCloudTopic, 30, &CameraNode::cameraCallback, this);
    segmentedCameraSub_ = nh_->subscribe("/stretch_pc/cluster", 30, &CameraNode::segmentedCameraCallback, this);
    pointPick_ = nh->advertise<geometry_msgs::PointStamped>("/clicked_point", 30);
    cameraPub_ = nh_->advertise<sensor_msgs::Image>("/stretch_gui/image", 30);
    cameraPointPub_ = nh_->advertise<sensor_msgs::Image>("/stretch_gui/imageSelection", 30);
    clickInitiated_ = nh_->advertise<std_msgs::Bool>("/stretch_gui/click_initiated", 30);

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

void CameraNode::sceneClicked(QPoint press, QPoint release, QSize screen) {
    sceneClickCloud_ = cloud_;
    if (!sceneClickCloud_ || sceneClickCloud_->size() == 0) {
        emit clickFailure();
        return;
    }
    int locX = press.x() * static_cast<double>(sceneClickCloud_->height) / static_cast<double>(screen.width());
    int locY = press.y() * static_cast<double>(sceneClickCloud_->width) / static_cast<double>(screen.height());

    try {
        if (locY > sceneClickCloud_->width || locX > sceneClickCloud_->height) {
            throw(std::runtime_error("Not in range"));
        }
        emit clickInitiated();
        pcl::PointXYZRGB p = sceneClickCloud_->at(locY, sceneClickCloud_->height - locX);

        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
            qDebug() << "click fail";
            throw(std::runtime_error("Point contains NaN"));
        }

        geometry_msgs::PointStamped::Ptr point(new geometry_msgs::PointStamped());
        point->header.frame_id = sceneClickCloud_->header.frame_id;

        point->point.x = p.x;
        point->point.y = p.y;
        point->point.z = p.z;

        try {
            emit distanceToTable(segmenter_->segmentAndFind(sceneClickCloud_, p, tfBuffer_));
        } catch (ObjectOutOfRange error) {
            ROS_INFO_STREAM("object out of range");
            emit invalidPoint();
            return;
        } catch (...) {
            ROS_INFO_STREAM("catch all");
            ROS_INFO_STREAM("Point cloud was empty after segmentation");
            emit clickFailure();
            return;
        }
        pointPick_.publish(point);
        emit clickSuccess();
    } catch (...) {
        emit clickFailure();
        return;
    }
}
