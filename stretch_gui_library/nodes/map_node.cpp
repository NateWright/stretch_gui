#include <ros/ros.h>

#include <stretch_gui_library/MapNode.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "Map_Node");
    ros::NodeHandlePtr nh(new ros::NodeHandle());
    MapNode mn(nh);
    ros::spin();
    return 0;
}
