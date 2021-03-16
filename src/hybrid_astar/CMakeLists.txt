cmake_minimum_required(VERSION 3.0.2)
project(hybrid_astar)


## C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/node4d.cpp
    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/node4d.h
    )

# Declare a catkin package
catkin_package()

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)
if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

include_directories(include ${catkin_INCLUDE_DIRS})

# Hybrid A* Header and Source files
add_executable(hybrid_astar src/hybrid_astar.cpp ${SOURCES} ${HEADERS})
target_link_libraries(hybrid_astar ${catkin_LIBRARIES})
target_link_libraries(hybrid_astar ${OMPL_LIBRARIES})