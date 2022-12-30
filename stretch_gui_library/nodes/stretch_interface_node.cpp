#include <ros/ros.h>

#include <stretch_gui_library/StretchInterface.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "Stretch_Interface_Node");
    ros::NodeHandlePtr nh(new ros::NodeHandle());
    StretchInterface sn(nh);
    ros::spin();
    return 0;
}
