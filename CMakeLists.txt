cmake_minimum_required(VERSION 3.5)
project(pneudrive)

# # Enable C++17 support, required for ROS 2
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# # Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

# optimize to -O2 for speed, but keep footprint it small
set(CMAKE_CXX_FLAGS "-Os")


set(WIRINGPI_INCLUDE_DIRS "/usr/include/wiringpi2/")
set(WIRINGPI_LIB "/usr/lib/aarch64-linux-gnu/libwiringPi.so")
#include_directories(${WIRINGPI_INCLUDE_DIRS})

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rad_msgs REQUIRED)


# ##################################
# # catkin specific configuration ##
# ##################################
# catkin_package(
# INCLUDE_DIRS
# CATKIN_DEPENDS message_runtime
# rad_msgs
# )

# For profilling
# add_compile_options(-pg)
# set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

# ##########
# # Build ##
# ##########
# include_directories(
# include
# ${catkin_INCLUDE_DIRS}
# ${WIRINGPI_INCLUDE_DIRS}
# )
include_directories(include)

add_executable(pressure_controller_node src/rs485_pressure_control.cpp src/PressureController.cpp)
target_include_directories(pressure_controller_node PRIVATE ${WIRINGPI_INCLUDE_DIRS})
target_link_libraries(pressure_controller_node ${WIRINGPI_LIB})

ament_target_dependencies(pressure_controller_node
  rclcpp
  tf2
  rclpy
  sensor_msgs
  std_msgs
  rad_msgs
)

install(TARGETS
  pressure_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# add dependency on rad_msgs
# add_dependencies(pressure_controller_node ${rad_msgs_EXPORTED_TARGETS})

# Auto-generates configuration files for other packages, replaces catkin_package()
ament_package()
