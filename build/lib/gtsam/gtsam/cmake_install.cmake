# Install script for directory: /Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam" TYPE FILE FILES
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/global_includes.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/precompiled_header.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam" TYPE FILE FILES
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/config.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/dllexport.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/libgtsam.4.2.0.dylib"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/libgtsam.4.dylib"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.4.2.0.dylib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.4.dylib"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      execute_process(COMMAND "/usr/bin/install_name_tool"
        -id "/usr/local/lib/libgtsam.4.dylib"
        -change "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/3rdparty/metis/libmetis/libmetis-gtsam.dylib" "/usr/local/lib/libmetis-gtsam.dylib"
        "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/strip" -x "${file}")
      endif()
    endif()
  endforeach()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/libgtsam.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.dylib")
    execute_process(COMMAND "/usr/bin/install_name_tool"
      -id "/usr/local/lib/libgtsam.4.dylib"
      -change "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/3rdparty/metis/libmetis/libmetis-gtsam.dylib" "/usr/local/lib/libmetis-gtsam.dylib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.dylib")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/3rdparty/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/base/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/basis/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/geometry/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/inference/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/symbolic/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/discrete/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/linear/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/nonlinear/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/sam/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/sfm/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/slam/cmake_install.cmake")
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/navigation/cmake_install.cmake")

endif()

