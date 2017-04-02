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
)

set(CMAKE_CXX_FLAGS "-std=c++11")

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
)

add_library(${PROJECT_NAME} src/VehicleOctomapServer.cpp src/QuadrotorCommand.cpp src/IterativePlanner.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

add_executable(target_tracking_node src/target_tracking_node.cpp)
target_link_libraries(target_tracking_node ${PROJECT_NAME} ${LINK_LIBS})

add_executable(uav_teleop_keyboard src/uav_teleop_keyboard.cpp)
target_link_libraries(uav_teleop_keyboard ${PROJECT_NAME} ${LINK_LIBS})
