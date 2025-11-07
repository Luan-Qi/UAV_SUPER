cmake_minimum_required(VERSION 3.0.2)
project(uav_locatiation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  tf
  Eigen3
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uav_planner
#  CATKIN_DEPENDS geometry_msg nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include/uav_locatiation
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(fast_lio_localization src/global_localization.cpp)
target_link_libraries(fast_lio_localization ${catkin_LIBRARIES})
add_dependencies(fast_lio_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(publish_initial_pose src/publish_initial_pose.cpp)
target_link_libraries(publish_initial_pose ${catkin_LIBRARIES})
add_dependencies(publish_initial_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(transform_fusion src/transform_fusion.cpp)
target_link_libraries(transform_fusion ${catkin_LIBRARIES})
add_dependencies(transform_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
