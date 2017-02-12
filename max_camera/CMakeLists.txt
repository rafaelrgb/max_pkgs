cmake_minimum_required(VERSION 2.8.3)
project(max_camera)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(image_subscriber src/image_subscriber.cpp)
add_dependencies(image_subscriber ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_subscriber ${catkin_LIBRARIES})

add_executable(image_publisher src/image_publisher.cpp)
add_dependencies(image_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_publisher ${catkin_LIBRARIES})
