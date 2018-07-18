# Install script for directory: /home/inavars/Desktop/projects/SparseSeg/ros/src/vlp-16_explorer

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/inavars/Desktop/projects/SparseSeg/ros/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/inavars/Desktop/projects/SparseSeg/ros/build/vlp-16_explorer/catkin_generated/installspace/groundFilter.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/groundFilter/cmake" TYPE FILE FILES
    "/home/inavars/Desktop/projects/SparseSeg/ros/build/vlp-16_explorer/catkin_generated/installspace/groundFilterConfig.cmake"
    "/home/inavars/Desktop/projects/SparseSeg/ros/build/vlp-16_explorer/catkin_generated/installspace/groundFilterConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/groundFilter" TYPE FILE FILES "/home/inavars/Desktop/projects/SparseSeg/ros/src/vlp-16_explorer/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/inavars/Desktop/projects/SparseSeg/ros/build/vlp-16_explorer/catkin_generated/installspace/groundFilter.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/groundFilter/cmake" TYPE FILE FILES
    "/home/inavars/Desktop/projects/SparseSeg/ros/build/vlp-16_explorer/catkin_generated/installspace/groundFilterConfig.cmake"
    "/home/inavars/Desktop/projects/SparseSeg/ros/build/vlp-16_explorer/catkin_generated/installspace/groundFilterConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/groundFilter" TYPE FILE FILES "/home/inavars/Desktop/projects/SparseSeg/ros/src/vlp-16_explorer/package.xml")
endif()

