# Install script for directory: /home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tanujthakkar/ROS/catkin_ws/build/ridgeback_manipulation/ridgeback_kinova_description/catkin_generated/installspace/ridgeback_kinova_description.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ridgeback_kinova_description/cmake" TYPE FILE FILES
    "/home/tanujthakkar/ROS/catkin_ws/build/ridgeback_manipulation/ridgeback_kinova_description/catkin_generated/installspace/ridgeback_kinova_descriptionConfig.cmake"
    "/home/tanujthakkar/ROS/catkin_ws/build/ridgeback_manipulation/ridgeback_kinova_description/catkin_generated/installspace/ridgeback_kinova_descriptionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ridgeback_kinova_description" TYPE FILE FILES "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ridgeback_kinova_description" TYPE DIRECTORY FILES "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/urdf")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ridgeback_kinova_description" TYPE PROGRAM FILES
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_dual_j2n6s200_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_dual_j2n6s300_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_dual_j2n7s300_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_dual_j2s6s300_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_dual_j2s7s300_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_dual_m1n6s200_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_dual_m1n6s300_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_j2n6s200_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_j2n6s300_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_j2n7s300_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_j2s6s300_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_j2s7s300_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_m1n6s200_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_m1n6s300_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_dual_gen3_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_dual_gen3_robotiq_2f_85_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_dual_gen3_robotiq_2f_140_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_gen3_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_gen3_robotiq_2f_85_envar"
    "/home/tanujthakkar/ROS/catkin_ws/src/ridgeback_manipulation/ridgeback_kinova_description/scripts/setup_ridgeback_gen3_robotiq_2f_140_envar"
    )
endif()

