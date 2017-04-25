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
  std_msgs
  nav_msgs
  geometry_msgs
  image_transport
  message_generation
)

find_package(Boost COMPONENTS system thread date_time REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 2.2 REQUIRED)
find_package(PkgConfig REQUIRED)
FIND_PACKAGE(VTK REQUIRED)

pkg_check_modules(EIGEN REQUIRED eigen3)

catkin_package(
      INCLUDE_DIRS include 
      CATKIN_DEPENDS message_runtime
)

include_directories(include

  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

INCLUDE(${VTK_USE_FILE})
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${SERIAL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(tms_node src/tracker2.cpp)
target_link_libraries(tms_node  ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} ${VTK_LIBRARIES} )



