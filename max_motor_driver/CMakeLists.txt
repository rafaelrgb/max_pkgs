cmake_minimum_required(VERSION 2.8.3)
project(max_motor_driver)

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

add_executable(motor_driver src/motor_driver.cpp)
add_dependencies(motor_driver ${catkin_EXPORTED_TARGETS} wiringPi)
target_link_libraries(motor_driver ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})

add_executable(teste src/teste.cpp)
add_dependencies(teste ${catkin_EXPORTED_TARGETS} wiringPi)
target_link_libraries(teste ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY})
