#include <ros/ros.h>

#include <stretch_gui_library/GraspNode.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "Grasp_Node");
    ros::NodeHandlePtr nh(new ros::NodeHandle());
    ros::AsyncSpinner s(2);
    GraspNode gn(nh);
    s.start();
    ros::waitForShutdown();
    return 0;
}
