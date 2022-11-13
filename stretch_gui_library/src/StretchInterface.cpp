#include "StretchInterface.hpp"

StretchInterface::StretchInterface(ros::NodeHandlePtr nh) : nh_(nh), panAngle_(0), tiltAngle_(0) {
    headTilt_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/head_tilt");
    headPan_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/head_pan");
    armLift_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/lift");
    armExtension_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/lift_extension");
    gipperYaw_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/wrist_yaw");
    gripperAperture_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/gripper_opening");
    moveToThread(this);
}

StretchInterface::~StretchInterface() {
    spinner_->stop();
    delete spinner_;
}

void StretchInterface::run() {
    spinner_ = new ros::AsyncSpinner(0);
    spinner_->start();
    exec();
}

std::pair<int, int> StretchInterface::getHeadPanTilt() {
    return {panAngle_, tiltAngle_};
}

void StretchInterface::headSetRotation(const double degPan, const double degTilt) {
    headSetPan(degPan);
    headSetTilt(degTilt);
}

void StretchInterface::headSetPan(const double degPan) {
    panAngle_ = degPan;
    stretch_gui_library::DoubleBool srv;
    srv.request.data = degPan * toRadians;
    headPan_.call(srv);
}
void StretchInterface::headSetTilt(const double degTilt) {
    tiltAngle_ = degTilt;
    stretch_gui_library::DoubleBool srv;
    srv.request.data = degTilt * toRadians;
    headTilt_.call(srv);
}
void StretchInterface::armSetHeight(const double metersHeight) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = metersHeight;
    armLift_.call(srv);
}
void StretchInterface::armSetReach(const double metersReach) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = metersReach;
    armExtension_.call(srv);
}
void StretchInterface::gripperSetRotate(const double deg) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = deg * toRadians;
    gipperYaw_.call(srv);
}
void StretchInterface::gripperSetGrip(const double deg) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = deg * toRadians;
    gripperAperture_.call(srv);
}
void StretchInterface::homeRobot() {
    headSetTilt();
    headSetPan();
    gripperSetGrip();
    gripperSetRotate();
    armSetHeight();
    armSetReach();
}

void StretchInterface::headUp() {
    tiltAngle_ += 5;
    headSetTilt(tiltAngle_);
}
void StretchInterface::headDown() {
    tiltAngle_ -= 5;
    headSetTilt(tiltAngle_);
}
void StretchInterface::headLeft() {
    panAngle_ += 5;
    headSetPan(panAngle_);
}
void StretchInterface::headRight() {
    panAngle_ -= 5;
    headSetPan(panAngle_);
}

void StretchInterface::headHome() {
    headSetRotation();
}