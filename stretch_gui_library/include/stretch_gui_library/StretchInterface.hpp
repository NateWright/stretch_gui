#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <stretch_gui_library/Double.h>
#include <stretch_gui_library/DoubleBool.h>

const double toRadians = M_PI / 180;

class StretchInterface {
   public:
    explicit StretchInterface(ros::NodeHandlePtr nh);
    ~StretchInterface();

    std::pair<int, int> getHeadPanTilt();

   private:
    ros::NodeHandlePtr nh_;
    ros::ServiceClient headTilt_;
    ros::ServiceClient headPan_;
    ros::ServiceClient armLift_;
    ros::ServiceClient armExtension_;
    ros::ServiceClient gipperYaw_;
    ros::ServiceClient gripperAperture_;

    int panAngle_;
    int tiltAngle_;

    ros::ServiceServer setHeadPan_;
    ros::ServiceServer setHeadTilt_;
    ros::ServiceServer setArmHeight_;
    ros::ServiceServer setArmReach_;
    ros::ServiceServer setGripperGrip_;
    ros::ServiceServer setGripperRotation_;
    ros::ServiceServer homeRobot_;
    ros::ServiceServer headUp_;
    ros::ServiceServer headDown_;
    ros::ServiceServer headLeft_;
    ros::ServiceServer headRight_;
    ros::ServiceServer headHome_;

    bool setHeadPan(stretch_gui_library::Double::Request&, stretch_gui_library::Double::Response&);
    bool setHeadTilt(stretch_gui_library::Double::Request&, stretch_gui_library::Double::Response&);
    bool setArmHeight(stretch_gui_library::Double::Request&, stretch_gui_library::Double::Response&);
    bool setArmReach(stretch_gui_library::Double::Request&, stretch_gui_library::Double::Response&);
    bool setGripperRotation(stretch_gui_library::Double::Request&, stretch_gui_library::Double::Response&);
    bool setGripperGrip(stretch_gui_library::Double::Request&, stretch_gui_library::Double::Response&);
    bool homeRobot(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool headUp(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool headDown(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool headLeft(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool headRight(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool headHome(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
};