cmake_minimum_required(VERSION 3.0.2)
project(uav_tracker)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosmsg
    rospy
    cv_bridge
    image_transport
    sensor_msgs
    std_msgs
    geometry_msgs
    mavros_msgs
    dynamic_reconfigure
    tf2
)

find_package(OpenCV REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/AstraColorHSV.cfg
)

catkin_package(
)

link_directories(${OpenCV_LIB_DIR})

include_directories(
    include/uav_tracker
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_subdirectory(include/uav_tracker)
add_executable(KCFTracker_node src/KCF_Tracker.cpp)
target_link_libraries(KCFTracker_node kcftracker ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(KCFTracker_control_node src/KCF_Tracker_control.cpp)
target_link_libraries(KCFTracker_control_node ${catkin_LIBRARIES})

