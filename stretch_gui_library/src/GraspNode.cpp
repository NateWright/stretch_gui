#include "GraspNode.hpp"

GraspNode::GraspNode(ros::NodeHandlePtr nh) : nh_(nh), robotMoving_(false), stopReplace_(false) {
    setMapping_ = nh_->serviceClient<stretch_gui_library::SetMapping>("/stretch_gui/set_mapping");
    setHeadPan_ = nh_->serviceClient<stretch_gui_library::Double>("/stretch_gui/set_head_pan");
    setHeadTilt_ = nh_->serviceClient<stretch_gui_library::Double>("/stretch_gui/set_head_tilt");
    setArmHeight_ = nh_->serviceClient<stretch_gui_library::Double>("/stretch_gui/set_arm_height");
    setArmReach_ = nh_->serviceClient<stretch_gui_library::Double>("/stretch_gui/set_arm_reach");
    setGripperRotation_ = nh_->serviceClient<stretch_gui_library::Double>("/stretch_gui/set_gripper_rotation");
    setGripperGrip_ = nh_->serviceClient<stretch_gui_library::Double>("/stretch_gui/set_gripper_grip");

    cmdVel_ = nh_->advertise<geometry_msgs::Twist>("/stretch/cmd_vel", 30);
    navigateRobot_ = nh_->advertise<geometry_msgs::PoseStamped>("/stretch_gui/navigate", 30);
    graspCompleted_ = nh_->advertise<std_msgs::Empty>("/stretch_gui/grasp_status", 30, true);
    hasObject_ = nh_->advertise<std_msgs::Bool>("/stretch_gui/has_object", 30, true);
    canNavigate_ = nh_->advertise<std_msgs::Bool>("/stretch_gui/can_navigate", 30, true);

    centerPointSub_ = nh_->subscribe("/stretch_pc/centerPoint", 30, &GraspNode::centerPointCallback, this);
    moving_ = nh_->subscribe("/stretch_gui/moving", 30, &GraspNode::movingCallback, this);

    setObjectOrientation_ = nh_->advertiseService("/stretch_gui/set_object_orientation", &GraspNode::setOrientation, this);

    nh_->getParam("/stretch_gui/verticalOffset", verticalOffset_);
    nh_->getParam("/stretch_gui/horizontalOffset", horizontalOffset_);

    pointBaseLink_.reset(new geometry_msgs::PointStamped());

    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
}

GraspNode::~GraspNode() {
    spinner_->stop();
    delete spinner_;
    delete tfListener_;
}

void GraspNode::centerPointCallback(const geometry_msgs::PointStamped::ConstPtr& input) {
    geometry_msgs::PointStamped point = tfBuffer_.transform(*input, "base_link");
    *pointBaseLink_ = point;
}

void GraspNode::lineUpCallback() {
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

bool GraspNode::setOrientation(stretch_gui_library::SetObjectOrientation::Request& request, stretch_gui_library::SetObjectOrientation::Response& response) {
    if (request.vertical) {
        orientation_ = VERTICAL;
    } else {
        orientation_ = HORIZONTAL;
    }
    return true;
}
void GraspNode::lineUpOffset(double offset) {
    ros::AsyncSpinner s(1);
    s.start();
    ros::Duration d(0.5);
    setMapping(false);

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
    setHeadPan(-90);
    d.sleep();
    setArmHeight(pointBaseLink_->point.z + 0.05);
    d.sleep();
    setGripperRotation(0);
    d.sleep();
    setGripperGrip(30);
    d.sleep();
    d.sleep();
    d.sleep();
    setArmReach(sqrt(pointBaseLink_->point.x * pointBaseLink_->point.x + pointBaseLink_->point.y * pointBaseLink_->point.y) - offset);
    d.sleep();
    d.sleep();
    d.sleep();
    setArmHeight(pointBaseLink_->point.z);

    graspDone();
    canNavigate(false);
}

void GraspNode::replaceObjectCallback() {
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
    setMapping(true);

    navigateRobot_.publish(homePose_);
    d.sleep();
    d.sleep();

    while (robotMoving_) {
        d.sleep();
        if (stopReplace_) {
            stopReplace_ = false;
            return;
        }
    }
    setMapping(false);

    cmdMsg_.angular.z = -cmdMsg_.angular.z;
    ros::Duration turnTime(turnTime_);
    ros::Timer timer = nh_->createTimer(
        ros::Duration(0.1), [&](const ros::TimerEvent& event) { cmdVel_.publish(cmdMsg_); });
    if (turnTime.sleep()) {
        timer.stop();
    }

    d.sleep();
    setHeadPan(-90);
    d.sleep();

    setArmHeight(pointBaseLink_->point.z + 0.05);
    d.sleep();
    setGripperRotation(0);
    d.sleep();
    setArmReach(sqrt(pointBaseLink_->point.x * pointBaseLink_->point.x + pointBaseLink_->point.y * pointBaseLink_->point.y) - offset);
    d.sleep();
    setArmHeight(pointBaseLink_->point.z);
    d.sleep();
    d.sleep();
    setGripperGrip(30);
    d.sleep();
    d.sleep();
    d.sleep();
    setArmHeight(pointBaseLink_->point.z + 0.05);

    hasObject(false);
}

void GraspNode::releaseObjectCallback() {
    ros::Duration d(3.0);
    setArmHeight(0.75);
    d.sleep();
    setGripperGrip(30);
    d.sleep();
    setArmReach();
    d.sleep();
    setHeadPan();
    d.sleep();
    setGripperGrip();
    d.sleep();
    setGripperRotation();
    d.sleep();
    setArmHeight();
    d.sleep();
    setMapping(true);
    d.sleep();
    canNavigate(true);
    hasObject(false);
}

void GraspNode::stowObjectCallback() {
    hasObject(true);
    ros::Duration d(1.0);
    setGripperGrip(-3);
    d.sleep();
    setArmHeight(pointBaseLink_->point.z + 0.05);
    d.sleep();
    setArmReach();
    d.sleep();
    setGripperRotation(90);
    d.sleep();
    setHeadTilt(-30);
    d.sleep();
    setHeadPan();
    d.sleep();
    setArmHeight(0.40);
    d.sleep();

    cmdMsg_.angular.z = -cmdMsg_.angular.z;
    ros::Duration turnTime(turnTime_);
    ros::Timer timer = nh_->createTimer(
        ros::Duration(0.1), [&](const ros::TimerEvent& event) { cmdVel_.publish(cmdMsg_); });
    if (turnTime.sleep()) {
        timer.stop();
    }
    setMapping(true);
    canNavigate(true);
}

void GraspNode::homeCallback() {
    ros::Duration d(0.25);
    setArmHeight(pointBaseLink_->point.z + 0.05);
    d.sleep();
    setArmReach();
    d.sleep();
    setHeadPan();
    d.sleep();
    setGripperGrip();
    d.sleep();
    setGripperRotation();
    d.sleep();
    setArmHeight();
    d.sleep();
    setMapping(true);
    d.sleep();
    canNavigate(true);
}