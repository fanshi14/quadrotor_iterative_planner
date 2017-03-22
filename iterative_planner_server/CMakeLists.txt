cmake_minimum_required(VERSION 2.8)
project(iterative_planner_server)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  dynamic_reconfigure
  nodelet
  octomap_server
  geometry_msgs
  quadrotor_trajectory
  bspline_ros
)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)
add_definitions(${EIGEN_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${QUADROTOR_TRAJECTORY_INCLUDE_DIRS}
  ${BSPLINE_ROS_INCLUDE_DIRS}
  ## tinyspline
  /home/shi/ros/shi_catkin_ws/build/bspline_ros/bspline/tinyspline/library
  #${PROJECT_BINARY_DIR}/bspline/tinyspline/library
  #/home/hawk/ros/jsk_mbzirc_task1_ws/build/bspline_ros/bspline/tinyspline/library
)

## Octomap not support c++11, tinyspline origin version requires c++11
## We need to delete related c++11 sentences in tinysplinecpp.h file.
## set(CMAKE_CXX_FLAGS "-std=c++11")

#generate_dynamic_reconfigure_options(cfg/OctomapServer.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS octomap PCL
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${QUADROTOR_TRAJECTORY_LIBRARIES}
  ${BSPLINE_ROS_LIBRARIES}
  #DEPENDS Eigen
)

add_library(${PROJECT_NAME} src/VehicleOctomapServer.cpp src/QuadrotorCommand.cpp src/IterativePlanner.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

add_executable(target_tracking_node src/target_tracking_node.cpp)
target_link_libraries(target_tracking_node
  ## tinyspline
  /home/shi/ros/shi_catkin_ws/build/bspline_ros/bspline/tinyspline/build/lib/libtinysplinecpp.so
  #${PROJECT_BINARY_DIR}/bspline/tinyspline/build/lib/libtinysplinecpp.so
  #/home/hawk/ros/jsk_mbzirc_task1_ws/build/bspline_ros/bspline/tinyspline/build/lib/libtinysplinecpp.so
  ${PROJECT_NAME} ${LINK_LIBS})

add_executable(uav_teleop_keyboard src/uav_teleop_keyboard.cpp)
target_link_libraries(uav_teleop_keyboard ${PROJECT_NAME} ${LINK_LIBS})

# Nodelet
#add_library(octomap_server_nodelet src/octomap_server_nodelet.cpp)
#target_link_libraries(octomap_server_nodelet ${PROJECT_NAME} ${LINK_LIBS})

# install targets:
install(TARGETS ${PROJECT_NAME}
  target_tracking_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})