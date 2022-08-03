#include "StretchInterfaceGazebo.hpp"

StretchInterfaceGazebo::StretchInterfaceGazebo(ros::NodeHandlePtr nh)
    : nh_(nh), headPan_(0), headTilt_(0), armLift_(0.2), armExtension_(0), gripperYaw_(M_PI), gripperAperture_(0), moveHead_(false), moveArm_(false), moveGripper_(false) {
    move_group_interface_arm_ = new moveit::planning_interface::MoveGroupInterface(ARM);
    move_group_interface_arm_->setGoalTolerance(0.01);
    move_group_interface_arm_->setPlanningTime(20.0);
    move_group_interface_arm_->setMaxVelocityScalingFactor(1.0);
    move_group_interface_arm_->setPlanningTime(5);

    move_group_interface_gripper_ = new moveit::planning_interface::MoveGroupInterface(GRIPPER);
    move_group_interface_gripper_->setGoalTolerance(0.01);
    move_group_interface_gripper_->setPlanningTime(20.0);
    move_group_interface_gripper_->setMaxVelocityScalingFactor(1.0);

    move_group_interface_head_ = new moveit::planning_interface::MoveGroupInterface(HEAD);
    move_group_interface_head_->setGoalTolerance(0.01);
    move_group_interface_head_->setPlanningTime(20.0);
    move_group_interface_head_->setMaxVelocityScalingFactor(1.0);

    subHandle_.reset(new ros::NodeHandle("stretch_interface_callback"));
    subHandle_->setCallbackQueue(&queue_);

    headTiltServiceServer_ = subHandle_->advertiseService("/stretch_interface/head_tilt", &StretchInterfaceGazebo::headTiltCallback, this);
    headPanServiceServer_ = subHandle_->advertiseService("/stretch_interface/head_pan", &StretchInterfaceGazebo::headPanCallback, this);
    armLiftServiceServer_ = subHandle_->advertiseService("/stretch_interface/lift", &StretchInterfaceGazebo::armLiftCallback, this);
    armExtensionServiceServer_ = subHandle_->advertiseService("/stretch_interface/lift_extension", &StretchInterfaceGazebo::armExtensionCallback, this);
    gripperYawServiceServer_ = subHandle_->advertiseService("/stretch_interface/wrist_yaw", &StretchInterfaceGazebo::gripperYawCallback, this);
    gripperApertureServiceServer_ = subHandle_->advertiseService("/stretch_interface/gripper_opening", &StretchInterfaceGazebo::gripperApertureCallback, this);
}
StretchInterfaceGazebo::~StretchInterfaceGazebo() {
    delete move_group_interface_arm_;
    delete move_group_interface_gripper_;
    delete move_group_interface_head_;
}

void StretchInterfaceGazebo::move() {
    queue_.callAvailable();
    std::lock_guard<std::mutex> guard(robotLock_);
    ros::AsyncSpinner s(1);
    s.start();
    if (moveHead_) {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::core::RobotStatePtr current_state = move_group_interface_head_->getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface_head_->getCurrentState()->getJointModelGroup(HEAD);
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions.at(0) = headPan_;
        joint_group_positions.at(1) = headTilt_;

        move_group_interface_head_->setJointValueTarget(joint_group_positions);
        move_group_interface_head_->plan(my_plan);
        move_group_interface_head_->execute(my_plan);
        moveHead_ = false;
    }
    if (moveArm_) {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::core::RobotStatePtr current_state = move_group_interface_arm_->getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface_arm_->getCurrentState()->getJointModelGroup(ARM);
        std::vector<double> joint_group_positions;
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions.at(0) = armLift_;
        double dist = armExtension_ / 4;
        for (int i = 1; i < 5; i++) {
            joint_group_positions.at(i) = dist;
        }
        joint_group_positions.back() = gripperYaw_;

        move_group_interface_arm_->setJointValueTarget(joint_group_positions);
        move_group_interface_arm_->plan(my_plan);
        move_group_interface_arm_->execute(my_plan);
        moveArm_ = false;
    }
    // if (moveGripper_) {
    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //     // Get current state
    //     moveit::core::RobotStatePtr gripper_state = move_group_interface_gripper_->getCurrentState();
    //     // Get joint values for the group
    //     const moveit::core::JointModelGroup* joint_model_group_gripper = move_group_interface_gripper_->getCurrentState()->getJointModelGroup(GRIPPER);
    //     std::vector<double> gripper_positions;
    //     gripper_state->copyJointGroupPositions(joint_model_group_gripper, gripper_positions);

    //     gripper_positions.front() = gripperAperture_;
    //     gripper_positions.back() = gripperAperture_;

    //     move_group_interface_gripper_->setJointValueTarget(gripper_positions);
    //     // Planning
    //     move_group_interface_gripper_->plan(my_plan);
    //     // Executing
    //     move_group_interface_gripper_->execute(my_plan);
    //     moveGripper_ = false;
    // }
}
bool StretchInterfaceGazebo::headTiltCallback(stretch_gui_library::DoubleBool::Request& req,
                                              stretch_gui_library::DoubleBool::Response& res) {
    std::lock_guard<std::mutex> guard(robotLock_);
    headTilt_ = req.data;
    moveHead_ = true;
    res.success = true;
    return true;
}
bool StretchInterfaceGazebo::headPanCallback(stretch_gui_library::DoubleBool::Request& req,
                                             stretch_gui_library::DoubleBool::Response& res) {
    std::lock_guard<std::mutex> guard(robotLock_);
    headPan_ = req.data;
    moveHead_ = true;
    res.success = true;
    return true;
}
bool StretchInterfaceGazebo::armLiftCallback(stretch_gui_library::DoubleBool::Request& req,
                                             stretch_gui_library::DoubleBool::Response& res) {
    std::lock_guard<std::mutex> guard(robotLock_);
    armLift_ = req.data;
    moveArm_ = true;
    res.success = true;
    return true;
}
bool StretchInterfaceGazebo::armExtensionCallback(stretch_gui_library::DoubleBool::Request& req,
                                                  stretch_gui_library::DoubleBool::Response& res) {
    std::lock_guard<std::mutex> guard(robotLock_);
    armExtension_ = req.data;
    moveArm_ = true;
    res.success = true;
    return true;
}
bool StretchInterfaceGazebo::gripperYawCallback(stretch_gui_library::DoubleBool::Request& req,
                                                stretch_gui_library::DoubleBool::Response& res) {
    std::lock_guard<std::mutex> guard(robotLock_);
    gripperYaw_ = req.data;
    moveArm_ = true;
    res.success = true;
    return true;
}
bool StretchInterfaceGazebo::gripperApertureCallback(stretch_gui_library::DoubleBool::Request& req,
                                                     stretch_gui_library::DoubleBool::Response& res) {
    std::lock_guard<std::mutex> guard(robotLock_);
    gripperAperture_ = req.data;
    moveGripper_ = true;
    res.success = true;
    return true;
}