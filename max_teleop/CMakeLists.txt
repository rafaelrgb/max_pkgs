cmake_minimum_required(VERSION 2.8.3)
project(max_teleop)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    max_msgs
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp max_msgs geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(H_FILES
    include/joystick.hh
)

set(CPP_FILES
    src/joystick.cc
)

add_executable(max_teleop_node src/max_teleop_node.cpp ${CPP_FILES})
add_dependencies(max_teleop_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(max_teleop_node ${catkin_LIBRARIES})
