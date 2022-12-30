#include <ros/ros.h>

#include <stretch_gui_library/CameraNode.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "Camera_Node");
    ros::NodeHandlePtr nh(new ros::NodeHandle());
    CameraNode cn(nh);
    ros::spin();
    return 0;
}
