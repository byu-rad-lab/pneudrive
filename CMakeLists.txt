cmake_minimum_required(VERSION 3.5)
project(pneudrive)

# # Enable C++17 support, required for ROS 2
set(CMAKE_CXX_STANDARD 17)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# optimize to -O2 for speed, but keep footprint it small
set(CMAKE_CXX_FLAGS "-Os")

# Set the path to WiringPi library and include directories
set(WIRINGPI_INCLUDE_DIRS "/usr/local/include/")
set(WIRINGPI_LIBRARIES "/usr/lib/libwiringPi.so")

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf REQUIRED)
find_package(rospy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rad_msgs REQUIRED)
find_package(message_generation REQUIRED)


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
# target_link_libraries(pressure_controller_node ${catkin_LIBRARIES} ${WIRINGPI_LIBRARIES})
ament_target_dependencies(pressure_controller_node
	rclcpp
	tf
	rospy
	sensor_msgs
	std_msgs
	rad_msgs
	message_generation
	)


# add dependency on rad_msgs
add_dependencies(pressure_controller_node ${catkin_EXPORTED_TARGETS})

# Auto-generates configuration files for other packages, replaces catkin_package()
ament_export_dependencies(message_runtime)
ament_export_dependencies(rad_msgs)
ament_package()
