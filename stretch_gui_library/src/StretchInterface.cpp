#include "StretchInterface.hpp"

StretchInterface::StretchInterface(ros::NodeHandlePtr nh) : nh_(nh), panAngle_(0), tiltAngle_(0) {
    headTilt_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/head_tilt");
    headPan_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/head_pan");
    armLift_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/lift");
    armExtension_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/lift_extension");
    gipperYaw_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/wrist_yaw");
    gripperAperture_ = nh_->serviceClient<stretch_gui_library::DoubleBool>("/stretch_interface/gripper_opening");

    setHeadPan_ = nh_->advertiseService("/stretch_gui/set_head_pan", &StretchInterface::setHeadPan, this);
    setHeadTilt_ = nh_->advertiseService("/stretch_gui/set_head_tilt", &StretchInterface::setHeadTilt, this);
    setArmHeight_ = nh_->advertiseService("/stretch_gui/set_arm_height", &StretchInterface::setArmHeight, this);
    setArmReach_ = nh_->advertiseService("/stretch_gui/set_arm_reach", &StretchInterface::setArmReach, this);
    setGripperGrip_ = nh_->advertiseService("/stretch_gui/set_gripper_grip", &StretchInterface::setGripperGrip, this);
    setGripperRotation_ = nh_->advertiseService("/stretch_gui/set_gripper_rotation", &StretchInterface::setGripperRotation, this);
    homeRobot_ = nh_->advertiseService("/stretch_gui/home_robot", &StretchInterface::homeRobot, this);
    headUp_ = nh_->advertiseService("/stretch_gui/head_up", &StretchInterface::headUp, this);
    headDown_ = nh_->advertiseService("/stretch_gui/head_down", &StretchInterface::headDown, this);
    headLeft_ = nh_->advertiseService("/stretch_gui/head_left", &StretchInterface::headLeft, this);
    headRight_ = nh_->advertiseService("/stretch_gui/head_right", &StretchInterface::headRight, this);
    headHome_ = nh_->advertiseService("/stretch_gui/head_home", &StretchInterface::headHome, this);
}

StretchInterface::~StretchInterface() {
}

std::pair<int, int> StretchInterface::getHeadPanTilt() {
    return {panAngle_, tiltAngle_};
}

bool StretchInterface::setHeadPan(stretch_gui_library::Double::Request& req, stretch_gui_library::Double::Response& res) {
    panAngle_ = req.data;
    stretch_gui_library::DoubleBool srv;
    srv.request.data = panAngle_ * toRadians;
    headPan_.call(srv);
    return true;
}
bool StretchInterface::setHeadTilt(stretch_gui_library::Double::Request& req, stretch_gui_library::Double::Response& res) {
    tiltAngle_ = req.data;
    stretch_gui_library::DoubleBool srv;
    srv.request.data = tiltAngle_ * toRadians;
    headTilt_.call(srv);
    return true;
}
bool StretchInterface::setArmHeight(stretch_gui_library::Double::Request& req, stretch_gui_library::Double::Response& res) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = req.data;
    armLift_.call(srv);
    return true;
}
bool StretchInterface::setArmReach(stretch_gui_library::Double::Request& req, stretch_gui_library::Double::Response& res) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = req.data;
    armExtension_.call(srv);
    return true;
}
bool StretchInterface::setGripperRotation(stretch_gui_library::Double::Request& req, stretch_gui_library::Double::Response& res) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = req.data * toRadians;
    gipperYaw_.call(srv);
    return true;
}
bool StretchInterface::setGripperGrip(stretch_gui_library::Double::Request& req, stretch_gui_library::Double::Response& res) {
    stretch_gui_library::DoubleBool srv;
    srv.request.data = req.data * toRadians;
    gripperAperture_.call(srv);
    return true;
}
bool StretchInterface::homeRobot(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    stretch_gui_library::Double::Request request;
    stretch_gui_library::Double::Response response;
    request.data = 0;
    setHeadTilt(request, response);
    setHeadPan(request, response);
    setGripperGrip(request, response);
    request.data = 180;
    setGripperRotation(request, response);
    request.data = 0.2;
    setArmHeight(request, response);
    request.data = 0;
    setArmReach(request, response);
    return true;
}

bool StretchInterface::headUp(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    stretch_gui_library::Double::Request request;
    stretch_gui_library::Double::Response response;
    tiltAngle_ += 5;
    request.data = tiltAngle_;
    setHeadTilt(request, response);
    return true;
}
bool StretchInterface::headDown(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    stretch_gui_library::Double::Request request;
    stretch_gui_library::Double::Response response;
    tiltAngle_ -= 5;
    request.data = tiltAngle_;
    setHeadTilt(request, response);
    return true;
}
bool StretchInterface::headLeft(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    stretch_gui_library::Double::Request request;
    stretch_gui_library::Double::Response response;
    panAngle_ += 5;
    request.data = panAngle_;
    setHeadPan(request, response);
    return true;
}
bool StretchInterface::headRight(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    stretch_gui_library::Double::Request request;
    stretch_gui_library::Double::Response response;
    panAngle_ -= 5;
    request.data = panAngle_;
    setHeadPan(request, response);
    return true;
}

bool StretchInterface::headHome(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    stretch_gui_library::Double::Request request;
    stretch_gui_library::Double::Response response;
    request.data = 0;
    setHeadPan(request, response);
    setHeadTilt(request, response);
    return true;
}