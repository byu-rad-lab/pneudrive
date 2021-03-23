cmake_minimum_required(VERSION 2.8.3)
project(byu_pressure_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -fno-stack-protector)

#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
#set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  rospy
  sensor_msgs
  std_msgs
  )

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS
)

###########
## Build ##
###########
include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_executable(i2c_pressure_control_node src/i2c_pressure_control.cpp src/I2CDevice.cpp src/GPIO.cpp src/PressureController.cpp)
target_link_libraries(i2c_pressure_control_node ${catkin_LIBRARIES})

#add_executable(i2c_imu_publisher_node src/i2c_imu_publisher.cpp src/I2CDevice.cpp src/IMUPublisher.cpp)
#target_link_libraries(i2c_imu_publisher_node ${catkin_LIBRARIES})
