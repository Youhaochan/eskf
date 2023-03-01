cmake_minimum_required(VERSION 3.0.2)
project(eskf)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)
find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
include_directories(/usr/local/include/yaml-cpp)

set(ALL_TARGET_LIBRARIES "")

# include(cmake/geographic.cmake)
include(cmake/glog.cmake)
include(cmake/PCL.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(eskf_library src/ESKF.cpp include/ESKF/ESKF.h)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES eskf
  CATKIN_DEPENDS roscpp rospy tf
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        )
add_executable(eskf src/ESKF_MAIN.cpp)
#add_dependencies(xtark_driver ${PROJECT_NAME}_gencfg)
# command below is used in the robot
# target_link_libraries(eskf ${catkin_LIBRARIES} ${Boost_LIBRARIES} libgflags.a libglog.a ${ALL_TARGET_LIBRARIES} eskf_library yaml-cpp) 
# command below is used in the pc
target_link_libraries(eskf ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${ALL_TARGET_LIBRARIES} eskf_library yaml-cpp)
