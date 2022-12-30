#include <ros/ros.h>

#include <stretch_gui_library/GraspNode.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "Grasp_Node");
    ros::NodeHandlePtr nh(new ros::NodeHandle());
    GraspNode gn(nh);
    ros::spin();
    return 0;
}
