#pragma once

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <stretch_gui_library/Double.h>
#include <stretch_gui_library/SetMapping.h>
#include <stretch_gui_library/SetObjectOrientation.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

class GraspNode {
   public:
    explicit GraspNode(ros::NodeHandlePtr nh);
    ~GraspNode();

   private:
    ros::NodeHandlePtr nh_;
    ros::Subscriber centerPointSub_;

    geometry_msgs::Twist cmdMsg_;
    double turnTime_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

    geometry_msgs::PointStamped::Ptr pointBaseLink_;
    geometry_msgs::PoseStamped::Ptr homePose_;

    double verticalOffset_;
    double horizontalOffset_;

    bool robotMoving_;
    bool stopReplace_;

    void centerPointCallback(const geometry_msgs::PointStamped::ConstPtr &input);
    void lineUpOffset(double offset);
    void replaceObjectOffset(double offset);

    ros::ServiceServer homeRobot_;
    bool homeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    ros::ServiceClient setMapping_;
    void setMapping(bool b) {
        stretch_gui_library::SetMapping msg;
        msg.request.mapping = b;
        setMapping_.call(msg);
    }
    ros::ServiceClient setHeadPan_;  // void headSetPan(double degPan = 0);
    void setHeadPan(double degPan = 0) {
        stretch_gui_library::Double msg;
        msg.request.data = degPan;
        setHeadPan_.call(msg);
    }

    ros::ServiceClient setHeadTilt_;  // void headSetTilt(double degTilt = 0);
    void setHeadTilt(double degTilt = 0) {
        stretch_gui_library::Double msg;
        msg.request.data = degTilt;
        setHeadTilt_.call(msg);
    }
    ros::ServiceClient setArmHeight_;  // void armSetHeight(double meters = 0.2);
    void setArmHeight(double mHeight = 0.2) {
        stretch_gui_library::Double msg;
        msg.request.data = mHeight;
        setArmHeight_.call(msg);
    }
    ros::ServiceClient setArmReach_;  // void armSetReach(double meters = 0);
    void setArmReach(double meters = 0) {
        stretch_gui_library::Double msg;
        msg.request.data = meters;
        setArmReach_.call(msg);
    }
    ros::ServiceClient setGripperRotation_;  // void gripperSetRotate(double deg = 180);
    void setGripperRotation(double deg = 180) {
        stretch_gui_library::Double msg;
        msg.request.data = deg;
        setGripperRotation_.call(msg);
    }
    ros::ServiceClient setGripperGrip_;  // void gripperSetGrip(double deg = 0);
    void setGripperGrip(double deg = 0) {
        stretch_gui_library::Double msg;
        msg.request.data = deg;
        setGripperGrip_.call(msg);
    }

    ros::Publisher cmdVel_;
    ros::Publisher navigateRobot_;   // void navigate(const geometry_msgs::PoseStamped::Ptr pose);
    ros::Publisher graspCompleted_;  // void graspDone(bool);
    void graspDone() {
        std_msgs::Empty msg;
        graspCompleted_.publish(msg);
    }
    ros::Publisher hasObject_;  // void hasObject(bool);
    void hasObject(bool b) {
        std_msgs::Bool msg;
        msg.data = b;
        hasObject_.publish(msg);
        nh_->setParam("/stretch_gui/has_object", b);
    }
    ros::Publisher canNavigate_;  // void canNavigate(bool);
    void canNavigate(bool b) {
        std_msgs::Bool msg;
        msg.data = b;
        canNavigate_.publish(msg);
        nh_->setParam("/stretch_gui/can_navigate", b);
    }
    // ros::Publisher releaseDone_;  // void releaseDone();
    ros::Subscriber moving_;  // bool moving();
    void movingCallback(std_msgs::Bool msg) {
        robotMoving_ = msg.data;
    }
    ros::Subscriber lineUp_;
    void lineUpCallback(std_msgs::Empty msg);
    ros::Subscriber stopAction_;
    void stopReplaceCallback() {
        stopReplace_ = true;
    }
    ros::ServiceServer stowObject_;
    bool stowObjectCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    ros::ServiceServer releaseObject_;
    bool releaseObjectCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    ros::ServiceServer replaceObject_;
    bool replaceObjectCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};
