################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(neuronbot2_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake        REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(serial             REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2                REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(nav_msgs           REQUIRED)
find_package(sensor_msgs        REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include include/
)

set(EXECUTABLE_NAME "neuronbot2_driver")

add_executable(${EXECUTABLE_NAME}
  src/main.cpp
  src/simple_dataframe_master.cpp
  src/neuron_serial.cpp
)
set(DEPENDENCIES
  "rclcpp"           
  "std_msgs"         
  "geometry_msgs" 
  "tf2_geometry_msgs"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "serial"
  "sensor_msgs"
)

ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES}
)
################################################################################
# Install
################################################################################
install(TARGETS ${EXECUTABLE_NAME}
 DESTINATION lib/${PROJECT_NAME})

 install(
  DIRECTORY launch cfg 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
