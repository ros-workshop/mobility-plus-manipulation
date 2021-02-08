cmake_minimum_required(VERSION 2.8.3)
project(husky_abb_manipulation)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_msgs
  tf
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  moveit_ros_planning
  gazebo_ros_link_attacher

  pluginlib 
)







catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES husky_abb_manipulation
#  CATKIN_DEPENDS moveit_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(husky_abb_grab_object src/husky_abb_grab_object.cpp)
target_link_libraries(husky_abb_grab_object ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
install(TARGETS husky_abb_grab_object DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_executable(transform_tag_frame src/transform_tag_frame.cpp)
target_link_libraries(transform_tag_frame ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
install(TARGETS transform_tag_frame DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


