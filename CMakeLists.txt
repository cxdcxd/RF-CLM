cmake_minimum_required(VERSION 2.8.3)
project(tms_node)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  genmsg
  pcl_conversions
  pcl_ros
  tf
  nav_msgs
  geometry_msgs
  message_generation
  message_runtime
  image_transport
  std_msgs
  sensor_msgs
  moveit_ros_planning_interface
)

find_package(Boost REQUIRED COMPONENTS filesystem system thread date_time REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(EIGEN REQUIRED eigen3)

add_definitions(${PCL_DEFINITIONS})

link_directories(${Boost_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
      INCLUDE_DIRS include 
      CATKIN_DEPENDS message_runtime
)

add_executable(tms_node src/tracker2.cpp)
add_dependencies(tms_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tms_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)


