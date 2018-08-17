cmake_minimum_required(VERSION 2.8.3)
project(prediction)

set (CMAKE_CXX_STANDARD 11)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -Wall -g -Wextra -Wno-unknown-pragmas")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -Wextra -werror -Wno-unknown-pragmas")

# Build deps
find_package(OpenCV)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    image_transport
    sensor_msgs
    std_msgs
    common_msgs
    path_planning_msgs
    perception_msgs
    common_msgs
    jsk_recognition_msgs
    )

generate_messages(
    DEPENDENCIES
    common_msgs
    path_planning_msgs
    perception_msgs
    )

# Runtime dependency
catkin_package(
    CATKIN_DEPENDS
    roscpp
    image_transport
    sensor_msgs
    std_msgs
    jsk_recognition_msgs
    perception_msgs
    path_planning_msgs
    common_msgs
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ./
    )

add_library(utils
    utils/utils.cpp
    )
target_link_libraries(utils
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
add_dependencies(utils ${catkin_EXPORTED_TARGETS})

add_library(tracker
    tracker/tracker.cpp
    )
target_link_libraries(tracker
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    utils
    )
add_dependencies(tracker ${catkin_EXPORTED_TARGETS})

add_executable(tracker_node
    tracker/main.cpp
    )
target_link_libraries(tracker_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIRBRARIES}
    tracker
    )

add_library(visualizer
    visualizer/visualizer.cpp
    )
target_link_libraries(visualizer
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    utils
    )
add_executable(visualizer_node
    visualizer/visualizer_node.cpp
    )
target_link_libraries(visualizer_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIRBRARIES}
    visualizer
    )

add_executable(lane_publisher_node
    lane_publisher/lane_publisher_node.cpp
    )
target_link_libraries(lane_publisher_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIRBRARIES}
    )
add_dependencies(lane_publisher_node ${catkin_EXPORTED_TARGETS})
