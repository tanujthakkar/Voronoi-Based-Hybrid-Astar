# Install script for directory: /home/tanujthakkar/ROS/catkin_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/tanujthakkar/ROS/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tanujthakkar/ROS/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tanujthakkar/ROS/catkin_ws/install" TYPE PROGRAM FILES "/home/tanujthakkar/ROS/catkin_ws/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tanujthakkar/ROS/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tanujthakkar/ROS/catkin_ws/install" TYPE PROGRAM FILES "/home/tanujthakkar/ROS/catkin_ws/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tanujthakkar/ROS/catkin_ws/install/setup.bash;/home/tanujthakkar/ROS/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tanujthakkar/ROS/catkin_ws/install" TYPE FILE FILES
    "/home/tanujthakkar/ROS/catkin_ws/build/catkin_generated/installspace/setup.bash"
    "/home/tanujthakkar/ROS/catkin_ws/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tanujthakkar/ROS/catkin_ws/install/setup.sh;/home/tanujthakkar/ROS/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tanujthakkar/ROS/catkin_ws/install" TYPE FILE FILES
    "/home/tanujthakkar/ROS/catkin_ws/build/catkin_generated/installspace/setup.sh"
    "/home/tanujthakkar/ROS/catkin_ws/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tanujthakkar/ROS/catkin_ws/install/setup.zsh;/home/tanujthakkar/ROS/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tanujthakkar/ROS/catkin_ws/install" TYPE FILE FILES
    "/home/tanujthakkar/ROS/catkin_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/tanujthakkar/ROS/catkin_ws/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/tanujthakkar/ROS/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/tanujthakkar/ROS/catkin_ws/install" TYPE FILE FILES "/home/tanujthakkar/ROS/catkin_ws/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/tanujthakkar/ROS/catkin_ws/build/gtest/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/ridgeback_desktop/ridgeback_desktop/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/ridgeback_simulator/ridgeback_simulator/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/ridgeback_simulator/mecanum_gazebo_plugin/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/ridgeback/ridgeback_msgs/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_msgs/tuw_airskin_msgs/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_msgs/tuw_msgs/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_multi_robot/tuw_multi_robot/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_multi_robot/tuw_multi_robot_demo/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_msgs/tuw_multi_robot_msgs/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_msgs/tuw_object_msgs/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/indoor_nav/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/car_geometric_planner/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/ridgeback/ridgeback_description/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/ridgeback_simulator/ridgeback_gazebo/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/ridgeback/ridgeback_navigation/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/ridgeback_desktop/ridgeback_viz/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/ridgeback_simulator/ridgeback_gazebo_plugins/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/ira_laser_tools/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_msgs/tuw_gazebo_msgs/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_geometry/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_msgs/tuw_geometry_msgs/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_msgs/tuw_local_controller_msgs/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_multi_robot/tuw_multi_robot_goal_generator/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_msgs/tuw_nav_msgs/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_multi_robot/tuw_multi_robot_ctrl/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_multi_robot/tuw_multi_robot_local_behavior_controller/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_msgs/tuw_vehicle_msgs/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_multi_robot/tuw_voronoi_graph/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_multi_robot/tuw_multi_robot_router/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_msgs/tuw_waypoint_to_spline_msgs/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/ridgeback/ridgeback_control/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/tuw_multi_robot/tuw_multi_robot_rviz/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/hybrid_astar/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/universal_global_planner/cmake_install.cmake")
  include("/home/tanujthakkar/ROS/catkin_ws/build/gazebo_ros_2Dmap_plugin/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/tanujthakkar/ROS/catkin_ws/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
