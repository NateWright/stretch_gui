#include "StretchMoveItInterface.hpp"

StretchMoveItInterface::StretchMoveItInterface(ros::NodeHandlePtr nh) : nh_(nh), panAngle_(0), tiltAngle_(0) {
    headTilt_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/head_tilt");
    headPan_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/head_pan");
    armLift_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/lift");
    armExtension_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/lift_extension");
    gipperYaw_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/wrist_yaw");
    gripperAperture_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/gripper_opening");
    moveToThread(this);
}

StretchMoveItInterface::~StretchMoveItInterface() {
    spinner_->stop();
    delete spinner_;
}

void StretchMoveItInterface::run() {
    spinner_ = new ros::AsyncSpinner(0);
    spinner_->start();
    exec();
}

std::pair<int, int> StretchMoveItInterface::getHeadPanTilt() {
    return {panAngle_, tiltAngle_};
}

void StretchMoveItInterface::headSetRotation(const double degPan, const double degTilt) {
    headSetPan(degPan);
    headSetTilt(degTilt);
}

void StretchMoveItInterface::headSetPan(const double degPan) {
    panAngle_ = degPan;
    stretch_gui_library::DoubleBool srv;
    srv.request.data = degPan * toRadians;
    headPan_.call(srv);
}
void StretchMoveItInterface::headSetTilt(const double degTilt) {
    tiltAngle_ = degTilt;
    stretch_gui_library::DoubleBool srv;
    srv.request.data = degTilt * toRadians;
    headTilt_.call(srv);
}
void StretchMoveItInterface::armSetHeight(const double metersHeight) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = metersHeight;
    armLift_.call(srv);
}
void StretchMoveItInterface::armSetReach(const double metersReach) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = metersReach;
    armExtension_.call(srv);
}
void StretchMoveItInterface::gripperSetRotate(const double deg) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = deg * toRadians;
    gipperYaw_.call(srv);
}
void StretchMoveItInterface::gripperSetGrip(const double deg) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = deg * toRadians;
    gripperAperture_.call(srv);
}
void StretchMoveItInterface::homeRobot() {
    headSetTilt();
    headSetPan();
    gripperSetGrip();
    gripperSetRotate();
    armSetHeight();
    armSetReach();
}

void StretchMoveItInterface::headUp() {
    tiltAngle_ += 5;
    headSetTilt(tiltAngle_);
}
void StretchMoveItInterface::headDown() {
    tiltAngle_ -= 5;
    headSetTilt(tiltAngle_);
}
void StretchMoveItInterface::headLeft() {
    panAngle_ += 5;
    headSetPan(panAngle_);
}
void StretchMoveItInterface::headRight() {
    panAngle_ -= 5;
    headSetPan(panAngle_);
}

void StretchMoveItInterface::headHome() {
    headSetRotation();
}