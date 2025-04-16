cmake_minimum_required(VERSION 3.8)
project(pneudrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Necessary for any ROS 2 package 
find_package(ament_cmake REQUIRED)

# Libaries specific for pneudrive, not all are necessarily used
# To add more, append 'find_package("package_name" REQUIRED)'
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rad_msgs REQUIRED)

# Adding library paths to point to for wiringpi
set(WIRINGPI_LINK_LIBS "/usr/local/lib/libwiringPi.so")
set(WIRINGPI_INCLUDE_DIRS "/usr/local/include/wiringpi2/")

# Link necessary library files
include_directories(include)
target_include_directories(pressure_controller_node PRIVATE ${WIRINGPI_INCLUDE_DIRS}) 
target_link_libraries(pressure_controller_node ${WIRINGPI_LINK_LIBS})

# Adding an executable for command line launch
add_executable(pressure_controller_node src/rs485_pressure_control.cpp src/PressureController.cpp)

# Add target dependencies, if more are needed, add them to the end of the list
ament_target_dependencies(pressure_controller_node
  rclcpp
  tf2
  rclpy
  sensor_msgs
  std_msgs
  rad_msgs
)

# Specifies where to save files created in the build process, needed to run
install(TARGETS
  pressure_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Standard lines from the default CMakeLists.txt, not needed but helps setup a linter for the package
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Necessary for any ROS 2 package
ament_package()

