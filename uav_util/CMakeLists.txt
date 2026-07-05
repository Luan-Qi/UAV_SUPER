cmake_minimum_required(VERSION 3.0.2)
project(uav_util)

# C++17 standard (required by PCL 1.12+ and std::filesystem)
add_compile_options(-std=c++17)

# ROS catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# --- Executables ---

add_executable(merge_cloud_node src/merge_cloud_node.cpp)
target_link_libraries(merge_cloud_node ${catkin_LIBRARIES})
add_dependencies(merge_cloud_node ${catkin_EXPORTED_TARGETS})

add_executable(pcd_downsample_node src/pcd_downsample_node.cpp)
target_link_libraries(pcd_downsample_node ${catkin_LIBRARIES} stdc++fs)
add_dependencies(pcd_downsample_node ${catkin_EXPORTED_TARGETS})

add_executable(simple_camera_driver src/simple_camera_driver.cpp)
target_link_libraries(simple_camera_driver ${catkin_LIBRARIES})
add_dependencies(simple_camera_driver ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})

add_executable(pcd_map_generator_node src/generate_pcd_map.cpp)
target_link_libraries(pcd_map_generator_node ${catkin_LIBRARIES})
add_dependencies(pcd_map_generator_node ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})

add_executable(imu_to_odom_node src/imu_to_odom/imu_to_odom_node.cpp)
target_link_libraries(imu_to_odom_node ${catkin_LIBRARIES})
add_dependencies(imu_to_odom_node ${catkin_EXPORTED_TARGETS})

add_executable(imu_health_check_node src/imu_to_odom/imu_health_check_node.cpp)
target_link_libraries(imu_health_check_node ${catkin_LIBRARIES})
add_dependencies(imu_health_check_node ${catkin_EXPORTED_TARGETS})
