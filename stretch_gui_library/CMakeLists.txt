cmake_minimum_required(VERSION 3.0.2)
project(stretch_gui_library)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  moveit_ros_planning_interface
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf2_ros
  cv_bridge
  std_msgs
  message_generation
)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets PrintSupport Concurrent)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PointClicked.msg
  PointStatus.msg
  Point.msg
  MapPose.msg
  MoveCommand.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DoubleBool.srv
  SetObjectOrientation.srv
  SetMapping.srv
  Double.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS moveit_ros_planning rviz_visual_tools moveit_ros_planning_interface pcl_ros roscpp rospy std_msgs tf2_ros cv_bridge message_generation
 DEPENDS Qt6Core Qt6PrintSupport Qt6Widgets
)

###########
## Build ##
###########

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

qt_wrap_cpp("include/${PROJECT_NAME}/*.hpp")

## Declare a C++ library
add_library(${PROJECT_NAME} STATIC
  include/${PROJECT_NAME}/GraspNode.hpp
  include/${PROJECT_NAME}/MapNode.hpp
  include/${PROJECT_NAME}/MoveBaseStatusNode.hpp
  include/${PROJECT_NAME}/CameraNode.hpp
  # include/${PROJECT_NAME}/sceneviewer.hpp
  include/${PROJECT_NAME}/StretchInterface.hpp
  include/${PROJECT_NAME}/ObjectSegmenter.hpp
  include/${PROJECT_NAME}/StretchInterfaceGazebo.hpp
  src/GraspNode.cpp
  src/MapNode.cpp
  src/MoveBaseStatusNode.cpp
  src/CameraNode.cpp
  src/StretchInterface.cpp
  src/ObjectSegmenter.cpp
  src/StretchInterfaceGazebo.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(stretch_gazebo nodes/stretch_gazebo.cpp)
add_executable(camera_node nodes/camera_node.cpp)
add_executable(grasp_node nodes/grasp_node.cpp)
add_executable(map_node nodes/map_node.cpp)
add_executable(move_base_node nodes/move_base_node.cpp)
add_executable(stretch_interface_node nodes/stretch_interface_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  PRIVATE
  ${catkin_LIBRARIES}
  Qt6::Widgets
  Qt6::PrintSupport
  Qt6::Core
  Qt6::Concurrent
)

target_link_libraries(stretch_gazebo ${PROJECT_NAME})
target_link_libraries(camera_node ${PROJECT_NAME})
target_link_libraries(grasp_node ${PROJECT_NAME})
target_link_libraries(map_node ${PROJECT_NAME})
target_link_libraries(move_base_node ${PROJECT_NAME})
target_link_libraries(stretch_interface_node ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
target_include_directories(${PROJECT_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_stretch_gui_library.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
