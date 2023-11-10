cmake_minimum_required(VERSION 3.0.2)
project(marsha_tests)

add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  length_penalty_metrics
  replanners_lib
  marsha
  roscpp
  human_simulator
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES marsha_tests
#  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simulation src/simulation.cpp)
add_dependencies(simulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulation
${catkin_LIBRARIES}
)
