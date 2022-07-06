# Install script for directory: /Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/basis

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/basis" TYPE FILE FILES
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/basis/Basis.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/basis/BasisFactors.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/basis/Chebyshev.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/basis/Chebyshev2.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/basis/FitBasis.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/basis/Fourier.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/basis/ParameterMatrix.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/basis/tests/cmake_install.cmake")

endif()

