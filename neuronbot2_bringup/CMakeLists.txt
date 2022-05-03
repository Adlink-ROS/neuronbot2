cmake_minimum_required(VERSION 2.8.3)
project(neuronbot2_bringup)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  dynamic_reconfigure
  neuronbot2_msgs
)
## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/neuronbot2_parameter.cfg
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
  include include/neuronbot2_bringup/
)

add_executable(neuronbot2_driver
  src/main.cpp
  src/base_driver_config.cpp
  src/base_driver.cpp
  src/data_holder.cpp
  src/simple_dataframe_master.cpp
  src/serial_transport.cpp
)
add_dependencies(neuronbot2_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(neuronbot2_driver
  ${catkin_LIBRARIES}
)
