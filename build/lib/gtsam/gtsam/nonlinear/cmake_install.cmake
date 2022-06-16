# Install script for directory: /Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/nonlinear" TYPE FILE FILES
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/AdaptAutoDiff.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/CustomFactor.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/DoglegOptimizer.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/DoglegOptimizerImpl.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/Expression-inl.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/Expression.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/ExpressionFactor.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/ExpressionFactorGraph.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/ExtendedKalmanFilter-inl.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/ExtendedKalmanFilter.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/FunctorizedFactor.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/GaussNewtonOptimizer.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/GncOptimizer.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/GncParams.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/GraphvizFormatting.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/ISAM2-impl.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/ISAM2.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/ISAM2Clique.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/ISAM2Params.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/ISAM2Result.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/ISAM2UpdateParams.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/LevenbergMarquardtParams.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/LinearContainerFactor.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/Marginals.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/NonlinearEquality.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/NonlinearFactor.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/NonlinearFactorGraph.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/NonlinearISAM.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/NonlinearOptimizer.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/NonlinearOptimizerParams.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/PriorFactor.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/Symbol.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/Values-inl.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/Values.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/WhiteNoiseFactor.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/expressionTesting.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/expressions.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/factorTesting.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/nonlinearExceptions.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/utilities.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/nonlinear/internal" TYPE FILE FILES
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/internal/CallRecord.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/internal/ExecutionTrace.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/internal/ExpressionNode.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/internal/JacobianMap.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/internal/LevenbergMarquardtState.h"
    "/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/lib/gtsam/gtsam/nonlinear/internal/NonlinearOptimizerState.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/ishitagoluguri/Desktop/MIT/UROP 2022/semantic_slam/build/lib/gtsam/gtsam/nonlinear/tests/cmake_install.cmake")

endif()

