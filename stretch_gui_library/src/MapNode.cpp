#include "MapNode.hpp"

MapNode::MapNode(ros::NodeHandlePtr nodeHandle)
    : nh_(nodeHandle), robotPos_(QPoint(0, 0)) {
    mapSub_ = nh_->subscribe("/map", 30, &MapNode::mapCallback, this);
    mapPointCloudSub_ = nh_->subscribe("/rtabmap/cloud_ground", 30, &MapNode::mapPointCloudCallback, this);
    std::string odomTopic;
    nh_->getParam("/stretch_gui/odom", odomTopic);
    movePub_ = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 30);
    mapPub_ = nh_->advertise<sensor_msgs::Image>("/stretch_gui/map", 30, true);
    posTimer_ = nh_->createTimer(ros::Duration(0.1), &MapNode::posCallback, this);
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    moveToThread(this);
}

MapNode::~MapNode() {
    posTimer_.stop();
    spinner_->stop();
    delete spinner_;
    delete tfListener_;
}

void MapNode::run() {
    spinner_ = new ros::AsyncSpinner(0);
    spinner_->start();
    posTimer_.start();
    exec();
}

void MapNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    mapMsg_ = msg;
}

void MapNode::mapPointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    ros::Duration d(1);
    nav_msgs::OccupancyGrid::ConstPtr msg = mapMsg_;
    while (!msg) {
        d.sleep();
        msg = mapMsg_;
    }
    const int width = msg->info.width,
              height = msg->info.height;

    mapSize_.setWidth(width);
    mapSize_.setHeight(height);
    resolution_ = msg->info.resolution;
    const int originX = width + msg->info.origin.position.x / resolution_,
              originY = -msg->info.origin.position.y / resolution_;

    origin_ = QPoint(originX, originY);

    cv::Mat mapImage(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    for (const auto& p : *cloud) {
        mapImage.at<cv::Vec3b>(cv::Point(originX - p.x / resolution_, originY + p.y / resolution_)) = {p.r, p.g, p.b};
    }
    cv_bridge::CvImage::Ptr map(new cv_bridge::CvImage());
    map->header.frame_id = cloud->header.frame_id;
    map->encoding = sensor_msgs::image_encodings::RGB8;
    map->image = mapImage;
    mapPub_.publish(map->toImageMsg());
}

void MapNode::posCallback(const ros::TimerEvent&) {
    std::string source = "map";
    std::string destination = "base_link";

    try {
        geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer_.lookupTransform(source, destination, ros::Time(0));

        robotRot_ = tf2::getYaw(transBaseLinkToMap.transform.rotation);

        robotPos_.setX(origin_.x() - transBaseLinkToMap.transform.translation.x / resolution_);
        robotPos_.setY(origin_.y() + transBaseLinkToMap.transform.translation.y / resolution_);
        emit robotPose(robotPos_, robotRot_);
    } catch (...) {
    }
}

void MapNode::moveRobot(QPoint press, QPoint release, QSize screen) {
    if (press == release) {
        return;
    }
    geometry_msgs::PoseStamped pose;

    QPoint mapLoc = translateScreenToMap(press, screen, mapSize_);

    double locX = (origin_.x() - mapLoc.x()) * resolution_,
           locY = (mapLoc.y() - origin_.y()) * resolution_;

    pose.header.frame_id = "map";
    pose.pose.position.x = locX;
    pose.pose.position.y = locY;

    double difX = release.x() - press.x();
    double difY = release.y() - press.y();

    tf2::Vector3 v1(-1, 0, 0);
    v1.normalize();
    tf2::Vector3 v2(difX, difY, 0);
    v2.normalize();

    tf2::Quaternion q = tf2::shortestArcQuat(v1, v2);
    q.setZ(-q.z());

    if (q.y() < 0 || q.y() > 0) {
        q.setX(0);
        q.setY(0);
        q.setZ(-1);
        q.setW(0);
    }

    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    movePub_.publish(pose);
}

void MapNode::moveRobotLoc(const geometry_msgs::PoseStamped::Ptr pose) {
    movePub_.publish(pose);
}

void MapNode::setHome() {
    try {
        std::string source = "map",
                    destination = "base_link";
        geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer_.lookupTransform(source, destination, ros::Time(0));

        robotHome_.header.frame_id = transBaseLinkToMap.header.frame_id;
        robotHome_.pose.orientation = transBaseLinkToMap.transform.rotation;
        robotHome_.pose.position.x = transBaseLinkToMap.transform.translation.x;
        robotHome_.pose.position.y = transBaseLinkToMap.transform.translation.y;
        robotHome_.pose.position.z = transBaseLinkToMap.transform.translation.z;
        emit homeSet(true);
    } catch (...) {
    }
}

void MapNode::setHomeIfNone() {
    std::string s = robotHome_.header.frame_id;
    if (s.length() == 0) {
        setHome();
    }
}

void MapNode::navigateHome() {
    movePub_.publish(robotHome_);
}
void MapNode::disableMapping() {
    std_srvs::Empty msg;
    ros::service::call("/rtabmap/pause", msg);
}
void MapNode::enableMapping() {
    std_srvs::Empty msg;
    ros::service::call("/rtabmap/resume", msg);
}

void MapNode::rotate(int degrees) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    tf2::Quaternion q;
    q.setRPY(0, 0, degrees * M_PI / 180);
    pose.pose.orientation = tf2::toMsg(q);
    movePub_.publish(pose);
}

void MapNode::rotateLeft(int degrees) {
    rotate(degrees);
}
void MapNode::rotateRight(int degrees) {
    rotate(-degrees);
}

void MapNode::drive(double meters) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = meters;
    pose.pose.orientation.w = 1;
    movePub_.publish(pose);
}

QPoint translateScreenToMap(QPoint p, QSize screen, QSize map) {
    return QPoint((double)p.x() * (double)map.width() / (double)screen.width(), (double)p.y() * (double)map.height() / (double)screen.height());
}
