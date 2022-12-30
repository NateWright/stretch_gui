#pragma once

#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

class MoveBaseStatus {
   public:
    explicit MoveBaseStatus(ros::NodeHandlePtr nodeHandle);
    ~MoveBaseStatus();

    ros::Publisher robotMoving_;
    ros::Subscriber stopRobot_;
    void stopRobotCallback(std_msgs::Empty msg);

   private:
    ros::NodeHandlePtr nh_;
    ros::Subscriber moveBaseStatusSub_;
    ros::Publisher moveBaseStopPub_;

    ros::AsyncSpinner *spinner_;

    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
};