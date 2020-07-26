cmake_minimum_required(VERSION 3.2)
project(particle_filter)

add_definitions(-std=c++11)

#########################################################################
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -fPIC")
else(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fPIC")
endif(CMAKE_BUILD_TYPE STREQUAL "Release")
#########################################################################

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ceres REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_executable(${PROJECT_NAME}
  src/ParticleFilter.cc
  src/main.cc
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${GLOG_LIBRARIES}
  gflags
)
