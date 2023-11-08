cmake_minimum_required(VERSION 3.0.2)
project(marsha_cell_safety)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  roscpp
  rosdyn_core
  subscription_notifier
  tf
  tf_conversions
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES marsha_cell_safety
 CATKIN_DEPENDS eigen_conversions tf_conversions geometry_msgs moveit_msgs roscpp rosdyn_core subscription_notifier tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(marsha_cell_safety src/pose_transform.cpp)
add_dependencies(${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(marsha_cell_safety
  ${catkin_LIBRARIES}
  )