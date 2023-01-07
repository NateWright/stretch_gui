#include "MapNode.hpp"

MapNode::MapNode(ros::NodeHandlePtr nodeHandle)
    : nh_(nodeHandle) {
    mapSub_ = nh_->subscribe("/map", 30, &MapNode::mapCallback, this);
    mapPointCloudSub_ = nh_->subscribe("/rtabmap/cloud_ground", 30, &MapNode::mapPointCloudCallback, this);
    moveRobotSub_ = nh_->subscribe("/stretch_gui/screen_move_robot", 30, &MapNode::moveRobotCallback, this);
    posTimer_ = nh_->createTimer(ros::Duration(0.1), &MapNode::posCallback, this);
    setHome_ = nh_->subscribe("/stretch_gui/set_home", 30, &MapNode::setHome, this);
    setHomeIfNone_ = nh_->subscribe("/stretch_gui/set_home_if_none", 30, &MapNode::setHomeIfNone, this);
    navigateHome_ = nh_->subscribe("/stretch_gui/navigate_home", 30, &MapNode::navigateHome, this);

    movePub_ = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 30);
    mapPub_ = nh_->advertise<sensor_msgs::CompressedImage>("/stretch_gui/map", 30, true);
    robotPose_ = nh_->advertise<stretch_gui_library::MapPose>("/stretch_gui/pose", 30, true);
    hasHome_ = nh_->advertise<std_msgs::Bool>("/stretch_gui/has_home", 30, true);

    setMapping_ = nh_->advertiseService("/stretch_gui/set_mapping", &MapNode::setMapping, this);
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
}

MapNode::~MapNode() {
    posTimer_.stop();
    spinner_->stop();
    delete spinner_;
    delete tfListener_;
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
    mapSize_.width = width;
    mapSize_.height = height;
    resolution_ = msg->info.resolution;
    const int originX = width + msg->info.origin.position.x / resolution_,
              originY = -msg->info.origin.position.y / resolution_;

    origin_.x = originX;
    origin_.y = originY;

    cv::Mat mapImage(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    for (const auto& p : *cloud) {
        mapImage.at<cv::Vec3b>(cv::Point(originX - p.x / resolution_, originY + p.y / resolution_)) = {p.r, p.g, p.b};
    }
    cv_bridge::CvImage::Ptr map(new cv_bridge::CvImage());
    map->header.frame_id = cloud->header.frame_id;
    map->encoding = sensor_msgs::image_encodings::RGB8;
    map->image = mapImage;
    mapPub_.publish(map->toCompressedImageMsg());
}

void MapNode::posCallback(const ros::TimerEvent&) {
    std::string source = "map";
    std::string destination = "base_link";

    try {
        geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer_.lookupTransform(source, destination, ros::Time(0));
        MapPose pose;
        pose.rotation = tf2::getYaw(transBaseLinkToMap.transform.rotation);

        pose.point.x = origin_.x - transBaseLinkToMap.transform.translation.x / resolution_;
        pose.point.y = origin_.y + transBaseLinkToMap.transform.translation.y / resolution_;
        robotPose_.publish(pose);
    } catch (...) {
    }
}

void MapNode::moveRobotCallback(stretch_gui_library::MoveCommand msg) {
    if (msg.p1.x == msg.p2.x && msg.p1.y == msg.p2.y) {
        return;
    }
    geometry_msgs::PoseStamped pose;

    Point mapLoc = translateScreenToMap(msg.p1, Size{msg.width, msg.height}, mapSize_);

    double locX = (origin_.x - mapLoc.x) * resolution_,
           locY = (mapLoc.y - origin_.y) * resolution_;

    pose.header.frame_id = "map";
    pose.pose.position.x = locX;
    pose.pose.position.y = locY;

    double difX = msg.p2.x - msg.p1.x;
    double difY = msg.p2.y - msg.p1.y;

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

void MapNode::setHome(std_msgs::Empty msg) {
    try {
        std::string source = "map",
                    destination = "base_link";
        geometry_msgs::TransformStamped transBaseLinkToMap = tfBuffer_.lookupTransform(source, destination, ros::Time(0));

        robotHome_.header.frame_id = transBaseLinkToMap.header.frame_id;
        robotHome_.pose.orientation = transBaseLinkToMap.transform.rotation;
        robotHome_.pose.position.x = transBaseLinkToMap.transform.translation.x;
        robotHome_.pose.position.y = transBaseLinkToMap.transform.translation.y;
        robotHome_.pose.position.z = transBaseLinkToMap.transform.translation.z;
        hasHome(true);
    } catch (...) {
    }
}

void MapNode::setHomeIfNone(std_msgs::Empty msg) {
    std::string s = robotHome_.header.frame_id;
    if (s.length() == 0) {
        setHome(msg);
    }
}
bool MapNode::setMapping(stretch_gui_library::SetMapping::Request& req, stretch_gui_library::SetMapping::Response& res) {
    if (req.mapping) {
        std_srvs::Empty msg;
        ros::service::call("/rtabmap/resume", msg);
    } else {
        std_srvs::Empty msg;
        ros::service::call("/rtabmap/pause", msg);
    }
    return true;
}

void MapNode::navigateHome(std_msgs::Empty msg) {
    movePub_.publish(robotHome_);
}

Point translateScreenToMap(Point p, Size screen, Size map) {
    Point point;
    point.x = static_cast<int>((double)p.x * (double)map.width / (double)screen.width);
    point.y = static_cast<int>((double)p.y * (double)map.height / (double)screen.height);
    return point;
}
