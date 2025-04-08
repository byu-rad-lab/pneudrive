cmake_minimum_required(VERSION 2.8.3)
project(pneudrive)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# optimize to -O2 for speed, but keep footprint it small
set(CMAKE_CXX_FLAGS "-Os")

# Set the path to WiringPi library and include directories, These are odroid specific install locations from an apt install.
# They will be consistent for the odroid N2+ but will be different on other SBCs.
set(WIRINGPI_INCLUDE_DIRS "/usr/include/wiringpi2")
set(WIRINGPI_LIBRARIES "/usr/lib/aarch64-linux-gnu/libwiringPi.so")

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  rad_msgs
  message_generation
)

# ##################################
# # catkin specific configuration ##
# ##################################
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS message_runtime
  rad_msgs
)

# For profilling
# add_compile_options(-pg)
# set(catkin_LIBRARIES ${catkin_LIBRARIES} -pg)

# ##########
# # Build ##
# ##########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${WIRINGPI_INCLUDE_DIRS}
)

add_executable(pressure_controller_node src/rs485_pressure_control.cpp src/PressureController.cpp)
target_link_libraries(pressure_controller_node ${catkin_LIBRARIES} ${WIRINGPI_LIBRARIES})

# add dependency on rad_msgs
add_dependencies(pressure_controller_node ${catkin_EXPORTED_TARGETS})