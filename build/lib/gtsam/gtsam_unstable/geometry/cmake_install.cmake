# Install script for directory: /Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam_unstable/geometry

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam_unstable/geometry" TYPE FILE FILES
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam_unstable/geometry/BearingS2.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam_unstable/geometry/Event.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam_unstable/geometry/InvDepthCamera3.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam_unstable/geometry/Pose3Upright.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam_unstable/geometry/SimPolygon2D.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam_unstable/geometry/SimWall2D.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam_unstable/geometry/tests/cmake_install.cmake")

endif()

