cmake_minimum_required(VERSION 2.8.3)
project(filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs geometry_msgs tf dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/MyStuff.cfg
  cfg/MyStuff2.cfg 
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES temp
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




add_executable(Mahony_filter src/Mahony_filter.cpp)
add_dependencies(Mahony_filter ${PROJECT_NAME}_gencfg)
target_link_libraries(Mahony_filter ${catkin_LIBRARIES} )


add_executable(Madgwick_filter src/Madgwick_filter.cpp)
add_dependencies(Madgwick_filter ${PROJECT_NAME}_gencfg)
target_link_libraries(Madgwick_filter ${catkin_LIBRARIES} )

add_executable(bias_calculator src/bias_calculator.cpp)
add_dependencies(bias_calculator ${PROJECT_NAME}_gencfg)
target_link_libraries(bias_calculator ${catkin_LIBRARIES} )


