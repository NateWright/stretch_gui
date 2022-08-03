#ifndef StretchInterfaceGazebo_HPP
#define StretchInterfaceGazebo_HPP

#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <mutex>

#include "stretch_gui_library/DoubleBool.h"

const std::string HEAD = "stretch_head",
                  ARM = "stretch_arm",
                  GRIPPER = "stretch_gripper";

class StretchInterfaceGazebo {
   public:
    explicit StretchInterfaceGazebo(ros::NodeHandlePtr nh);
    ~StretchInterfaceGazebo();

    void move();

   private:
    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr subHandle_;
    ros::CallbackQueue queue_;
    ros::ServiceServer headTiltServiceServer_;
    ros::ServiceServer headPanServiceServer_;
    ros::ServiceServer armLiftServiceServer_;
    ros::ServiceServer armExtensionServiceServer_;
    ros::ServiceServer gripperYawServiceServer_;
    ros::ServiceServer gripperApertureServiceServer_;

    moveit::planning_interface::MoveGroupInterface *move_group_interface_arm_;
    moveit::planning_interface::MoveGroupInterface *move_group_interface_head_;
    moveit::planning_interface::MoveGroupInterface *move_group_interface_gripper_;

    float headPan_,
        headTilt_,
        armLift_,
        armExtension_,
        gripperYaw_,
        gripperAperture_;
    bool moveHead_,
        moveArm_,
        moveGripper_;

    std::mutex robotLock_;

    bool headTiltCallback(stretch_gui_library::DoubleBool::Request &req,
                          stretch_gui_library::DoubleBool::Response &res);
    bool headPanCallback(stretch_gui_library::DoubleBool::Request &req,
                         stretch_gui_library::DoubleBool::Response &res);
    bool armLiftCallback(stretch_gui_library::DoubleBool::Request &req,
                         stretch_gui_library::DoubleBool::Response &res);
    bool armExtensionCallback(stretch_gui_library::DoubleBool::Request &req,
                              stretch_gui_library::DoubleBool::Response &res);
    bool gripperYawCallback(stretch_gui_library::DoubleBool::Request &req,
                            stretch_gui_library::DoubleBool::Response &res);
    bool gripperApertureCallback(stretch_gui_library::DoubleBool::Request &req,
                                 stretch_gui_library::DoubleBool::Response &res);
};

#endif  // StretchInterfaceGazebo_HPP
