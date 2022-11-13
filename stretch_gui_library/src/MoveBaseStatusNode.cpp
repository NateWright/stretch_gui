#include "MoveBaseStatusNode.hpp"

MoveBaseStatus::MoveBaseStatus(ros::NodeHandlePtr nodeHandle) : nh_(nodeHandle), moving_(false) {
    moveBaseStatusSub_ = nh_->subscribe("/move_base/status", 1000, &MoveBaseStatus::moveBaseStatusCallback, this);
    moveBaseStopPub_ = nh_->advertise<actionlib_msgs::GoalID>("/move_base/cancel", 30);
    moveToThread(this);
}

MoveBaseStatus::~MoveBaseStatus() {
    spinner_->stop();
    delete spinner_;
}

void MoveBaseStatus::run() {
    spinner_ = new ros::AsyncSpinner(0);
    spinner_->start();
    exec();
}

void MoveBaseStatus::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg) {
    if (!msg.get()->status_list.empty() && msg.get()->status_list.back().status == 1) {
        moving_ = true;
        robotMoving(true);
        return;
    }
    moving_ = false;
    robotMoving(false);
}

void MoveBaseStatus::stopRobot() {
    actionlib_msgs::GoalID stop;
    stop.stamp.sec = 0;
    stop.stamp.nsec = 0;
    stop.id = "";
    moveBaseStopPub_.publish(stop);
}