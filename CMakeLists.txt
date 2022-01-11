cmake_minimum_required(VERSION 3.0.2)
project(NewtonMethod)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

# For Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(NewtonMethod src/newtom_method.cpp )

target_link_libraries(OccupiedGridObstacleAvoidanceRos
  ${catkin_LIBRARIES}
)
