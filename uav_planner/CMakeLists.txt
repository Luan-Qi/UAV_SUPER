cmake_minimum_required(VERSION 3.0.2)
project(uav_planner)

set(octomap_DIR /home/orangepi/ros_catkin_ws/install_isolated/share/octomap)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  octomap
  octomap_msgs
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  Eigen3
  uav_px4_ctrl
)

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
#  INCLUDE_DIRS include
#  LIBRARIES uav_planner
#  CATKIN_DEPENDS geometry_msg nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include/uav_planner
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(astar_planner_node src/astar/astar_planner_node.cpp src/astar/astar_searcher.cpp src/astar/astar_visualization.cpp)
target_link_libraries(astar_planner_node ${catkin_LIBRARIES})
add_dependencies(astar_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(transform_path src/transform_path.cpp)
target_link_libraries(transform_path ${catkin_LIBRARIES})
add_dependencies(transform_path ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(path_follower src/simple_path_follower/simple_path_follower.cpp src/simple_path_follower/visual_path_follower.cpp)
target_link_libraries(path_follower ${catkin_LIBRARIES})
add_dependencies(path_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(fast_path_follower src/simple_path_follower/fast_path_follower.cpp src/simple_path_follower/visual_path_follower.cpp)
target_link_libraries(fast_path_follower ${catkin_LIBRARIES})
add_dependencies(fast_path_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(dp_path_simplify src/path_simplify/DP_path_simplify.cpp)
target_link_libraries(dp_path_simplify ${catkin_LIBRARIES})
add_dependencies(dp_path_simplify ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(greedy_path_simplify src/path_simplify/greedy_path_simplify.cpp)
target_link_libraries(greedy_path_simplify ${catkin_LIBRARIES})
add_dependencies(greedy_path_simplify ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(dc_path_smoothify src/path_smoothify/discrete_curve_path_smoothify.cpp)
target_link_libraries(dc_path_smoothify ${catkin_LIBRARIES})
add_dependencies(dc_path_smoothify ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(bezier_path_smoothify src/path_smoothify/bezier_path_smoothify.cpp)
target_link_libraries(bezier_path_smoothify ${catkin_LIBRARIES})
add_dependencies(bezier_path_smoothify ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
