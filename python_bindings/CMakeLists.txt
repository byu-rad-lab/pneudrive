cmake_minimum_required(VERSION 3.5)
project(pneudrive_py)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(Python3 COMPONENTS Interpreter Development)

# optimize to -O2 for speed, but keep footprint it small
set(CMAKE_CXX_FLAGS "-Os")

# Set the path to WiringPi library and include directories
set(WIRINGPI_INCLUDE_DIRS "/usr/local/include/")
set(WIRINGPI_LIBRARIES "/usr/lib/libwiringPi.so")
set(PYBIND11_PYTHON_VERSION 3.8)

include_directories(
    include
    ${WIRINGPI_INCLUDE_DIRS}
)

add_subdirectory(./extern/pybind11)
pybind11_add_module(pneudrive_py src/PressureControllerPython.cpp)
target_link_libraries(pneudrive_py PRIVATE ${WIRINGPI_LIBRARIES})