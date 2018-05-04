cmake_minimum_required(VERSION 2.8.3)
project(max_camera)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  sensor_msgs
)

catkin_package(
    CATKIN_DEPENDS cv_bridge image_transport sensor_msgs
)

find_package(OpenCV)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

#add_executable(image_subscriber src/image_subscriber.cpp)
#add_dependencies(image_subscriber ${catkin_EXPORTED_TARGETS})
#target_link_libraries(image_subscriber ${catkin_LIBRARIES})

#add_executable(image_publisher src/image_publisher.cpp)
#add_dependencies(image_publisher ${catkin_EXPORTED_TARGETS})
#target_link_libraries(image_publisher ${catkin_LIBRARIES})

add_executable(video_pub src/video_publisher.cpp)
add_dependencies(video_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(video_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
