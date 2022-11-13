#include <ros/ros.h>

#include <stretch_gui_library/StretchInterfaceGazebo.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "stretch_move_group_interface");
    ros::NodeHandlePtr nh(new ros::NodeHandle());

    StretchInterfaceGazebo stretch(nh);
    ros::Rate r(10);
    while (ros::ok()) {
        stretch.move();
        r.sleep();
    }

    return 0;
}