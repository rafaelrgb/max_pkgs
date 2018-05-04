cmake_minimum_required(VERSION 2.8.3)
project(max_velocity_sensor)

FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi /home/pi/wiringPi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velocity_sensor src/velocity_sensor.cpp)
add_dependencies(velocity_sensor ${catkin_EXPORTED_TARGETS} wiringPi)
target_link_libraries(velocity_sensor ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
