#include <ros/ros.h>

#include <stretch_gui_library/MoveBaseStatusNode.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "Move_Base_Status_Node");
    ros::NodeHandlePtr nh(new ros::NodeHandle());
    MoveBaseStatus mn(nh);
    ros::spin();
    return 0;
}
