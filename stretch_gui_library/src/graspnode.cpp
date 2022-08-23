#include "graspnode.hpp"

GraspNode::GraspNode(ros::NodeHandlePtr nh) : nh_(nh), robotMoving_(false), stopReplace_(false) {
    cmdVel_ = nh_->advertise<geometry_msgs::Twist>("/stretch/cmd_vel", 30);
    centerPointSub_ = nh_->subscribe("/stretch_pc/centerPoint", 30, &GraspNode::centerPointCallback, this);

    nh_->getParam("/stretch_gui/verticalOffset", verticalOffset_);
    nh_->getParam("/stretch_gui/horizontalOffset", horizontalOffset_);

    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    point_.reset(new geometry_msgs::PointStamped());
    moveToThread(this);
}

GraspNode::~GraspNode() {
    spinner_->stop();
    delete spinner_;
    delete tfListener_;
}

void GraspNode::run() {
    spinner_ = new ros::AsyncSpinner(0);
    spinner_->start();
    exec();
}

void GraspNode::centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& input) {
    geometry_msgs::PointStamped point = tfBuffer_.transform(*input, "map");
    point_.reset(new geometry_msgs::PointStamped());
    *point_ = point;
    point = tfBuffer_.transform(*input, "base_link");
    pointBaseLink_.reset(new geometry_msgs::PointStamped());
    *pointBaseLink_ = point;
}

void GraspNode::lineUp() {
    switch (orientation_) {
        case VERTICAL: {
            lineUpOffset(verticalOffset_);
            break;
        }
        case HORIZONTAL: {
            lineUpOffset(horizontalOffset_);
            break;
        }
    }
}

void GraspNode::setHorizontal() {
    orientation_ = HORIZONTAL;
}
void GraspNode::setVertical() {
    orientation_ = VERTICAL;
}
void GraspNode::lineUpOffset(double offset) {
    ros::AsyncSpinner s(1);
    s.start();
    ros::Duration d(0.5);
    emit disableMapping();

    std::string targetFrame = "map", sourceFrame = "base_link";

    geometry_msgs::TransformStamped transBaseToMap = tfBuffer_.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    homePose_.reset(new geometry_msgs::PoseStamped());
    homePose_->header.frame_id = "map";
    homePose_->pose.position.x = transBaseToMap.transform.translation.x;
    homePose_->pose.position.y = transBaseToMap.transform.translation.y;
    homePose_->pose.position.z = transBaseToMap.transform.translation.z;
    homePose_->pose.orientation = transBaseToMap.transform.rotation;

    geometry_msgs::PoseStamped::Ptr pose(new geometry_msgs::PoseStamped());

    pose->header.frame_id = "base_link";

    double angleRad = atan(pointBaseLink_->point.y / pointBaseLink_->point.x) + 88 * M_PI / 180;
    double speed = 0.25;
    if (angleRad >= 0) {
        cmdMsg_.angular.z = speed;
        turnTime_ = angleRad / speed;
        ros::Duration turnTime(turnTime_);
        ros::Timer timer = nh_->createTimer(
            ros::Duration(0.1), [&](const ros::TimerEvent& event) { cmdVel_.publish(cmdMsg_); });
        if (turnTime.sleep()) {
            timer.stop();
        }
    } else {
        cmdMsg_.angular.z = -speed;
        turnTime_ = -angleRad / speed;
        ros::Duration turnTime(turnTime_);
        ros::Timer timer = nh_->createTimer(
            ros::Duration(0.1), [&](const ros::TimerEvent& event) { cmdVel_.publish(cmdMsg_); });
        if (turnTime.sleep()) {
            timer.stop();
        }
    }

    d.sleep();
    emit headSetPan(-90);
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z + 0.05);
    d.sleep();
    emit gripperSetRotate(0);
    d.sleep();
    emit gripperSetGrip(30);
    d.sleep();
    d.sleep();
    d.sleep();
    emit armSetReach(sqrt(pointBaseLink_->point.x * pointBaseLink_->point.x + pointBaseLink_->point.y * pointBaseLink_->point.y) - offset);
    d.sleep();
    d.sleep();
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z);

    emit graspDone(true);
    emit canNavigate(false);
}

void GraspNode::replaceObject() {
    switch (orientation_) {
        case VERTICAL: {
            replaceObjectOffset(verticalOffset_);
            break;
        }
        case HORIZONTAL: {
            replaceObjectOffset(horizontalOffset_);
            break;
        }
    }
}

void GraspNode::replaceObjectOffset(double offset) {
    ros::AsyncSpinner s(1);
    s.start();
    ros::Duration d(1.0);
    emit enableMapping();

    emit navigate(homePose_);
    d.sleep();
    d.sleep();

    while (emit moving()) {
        d.sleep();
        if (stopReplace_) {
            stopReplace_ = false;
            return;
        }
    }
    emit disableMapping();

    cmdMsg_.angular.z = -cmdMsg_.angular.z;
    ros::Duration turnTime(turnTime_);
    ros::Timer timer = nh_->createTimer(
        ros::Duration(0.1), [&](const ros::TimerEvent& event) { cmdVel_.publish(cmdMsg_); });
    if (turnTime.sleep()) {
        timer.stop();
    }

    d.sleep();
    emit headSetPan(-90);
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z + 0.05);
    d.sleep();
    emit gripperSetRotate(0);
    d.sleep();
    emit armSetReach(sqrt(pointBaseLink_->point.x * pointBaseLink_->point.x + pointBaseLink_->point.y * pointBaseLink_->point.y) - offset);
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z);
    d.sleep();
    d.sleep();
    emit gripperSetGrip(30);
    d.sleep();
    d.sleep();
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z + 0.05);

    emit hasObject(false);
}

void GraspNode::releaseObject() {
    ros::Duration d(3.0);
    emit armSetHeight(0.75);
    d.sleep();
    emit gripperSetGrip(30);
    d.sleep();
    emit armSetReach();
    d.sleep();
    emit headSetPan();
    d.sleep();
    emit gripperSetGrip();
    d.sleep();
    emit gripperSetRotate();
    d.sleep();
    emit armSetHeight();
    d.sleep();
    emit enableMapping();
    d.sleep();
    emit canNavigate(true);
    emit hasObject(false);
    emit releaseDone();
}

void GraspNode::stowObject() {
    emit hasObject(true);
    ros::Duration d(1.0);
    emit gripperSetGrip(-3);
    d.sleep();
    emit armSetHeight(pointBaseLink_->point.z + 0.05);
    d.sleep();
    emit armSetReach();
    d.sleep();
    emit gripperSetRotate(90);
    d.sleep();
    emit headSetTilt(-30);
    d.sleep();
    emit headSetPan();
    d.sleep();
    emit armSetHeight(0.40);
    d.sleep();

    cmdMsg_.angular.z = -cmdMsg_.angular.z;
    ros::Duration turnTime(turnTime_);
    ros::Timer timer = nh_->createTimer(
        ros::Duration(0.1), [&](const ros::TimerEvent& event) { cmdVel_.publish(cmdMsg_); });
    if (turnTime.sleep()) {
        timer.stop();
    }
    emit enableMapping();
    emit canNavigate(true);
}

void GraspNode::home() {
    ros::Duration d(0.25);
    emit armSetHeight(pointBaseLink_->point.z + 0.05);
    d.sleep();
    emit armSetReach();
    d.sleep();
    emit headSetPan();
    d.sleep();
    emit gripperSetGrip();
    d.sleep();
    emit gripperSetRotate();
    d.sleep();
    emit armSetHeight();
    d.sleep();
    emit enableMapping();
    d.sleep();
    emit canNavigate(true);
}