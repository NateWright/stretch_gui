#pragma once

#include <ros/ros.h>

#include <QDebug>
#include <QObject>
#include <QThread>
#include <QTimer>

#include "stretch_gui_library/DoubleBool.h"

const double toRadians = M_PI / 180;

class StretchInterface : public QThread {
    Q_OBJECT
   public:
    explicit StretchInterface(ros::NodeHandlePtr nh);
    ~StretchInterface();
    void run() override;

    std::pair<int, int> getHeadPanTilt();

   private:
    ros::NodeHandlePtr nh_;
    ros::ServiceClient headTilt_;
    ros::ServiceClient headPan_;
    ros::ServiceClient armLift_;
    ros::ServiceClient armExtension_;
    ros::ServiceClient gipperYaw_;
    ros::ServiceClient gripperAperture_;

    ros::AsyncSpinner *spinner_;

    int panAngle_;
    int tiltAngle_;

   public slots:
    void headSetRotation(const double degPan = 0, const double degTilt = 0);
    void headSetPan(const double degPan = 0);
    void headSetTilt(const double degTilt = 0);
    void armSetHeight(const double metersHeight = 0.2);
    void armSetReach(const double metersReach = 0);
    void gripperSetRotate(const double deg = 180);
    void gripperSetGrip(const double deg = 0);
    void homeRobot();
    void headUp();
    void headDown();
    void headLeft();
    void headRight();
    void headHome();
};