#include "MoveBaseStatusNode.hpp"

MoveBaseStatus::MoveBaseStatus(ros::NodeHandlePtr nodeHandle) : nh_(nodeHandle) {
    moveBaseStatusSub_ = nh_->subscribe("/move_base/status", 1000, &MoveBaseStatus::moveBaseStatusCallback, this);
    moveBaseStopPub_ = nh_->advertise<actionlib_msgs::GoalID>("/move_base/cancel", 30);
    robotMoving_ = nh_->advertise<std_msgs::Bool>("/stretch_gui/moving", 30, true);
    stopRobot_ = nh_->subscribe("/stretch_gui/stop_robot", 30, &MoveBaseStatus::stopRobotCallback, this);
}

MoveBaseStatus::~MoveBaseStatus() {
    spinner_->stop();
    delete spinner_;
}

void MoveBaseStatus::moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg) {
    std_msgs::Bool b;
    if (!msg.get()->status_list.empty() && msg.get()->status_list.back().status == 1) {
        b.data = true;
        robotMoving_.publish(b);
        return;
    }
    b.data = false;
    robotMoving_.publish(b);
}

void MoveBaseStatus::stopRobotCallback(std_msgs::Empty msg) {
    actionlib_msgs::GoalID stop;
    stop.stamp.sec = 0;
    stop.stamp.nsec = 0;
    stop.id = "";
    moveBaseStopPub_.publish(stop);
}