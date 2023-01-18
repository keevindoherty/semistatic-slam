cmake_minimum_required(VERSION 3.0)
project(semantic_slam)

# new feature to Cmake Version > 2.8.12
# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

add_subdirectory(lib/gtsam)
add_subdirectory(lib/persistence_filter)
add_subdirectory(lib/hungarian-algorithm-cpp)

find_package(GTSAM 4.1 REQUIRED)
#find_package(PersistenceFilter REQUIRED)
find_package(matplotlibcpp REQUIRED)
find_package(hungarian REQUIRED)

set(SLAM_LIBS
  persistence_filter
  gtsam
  matplotlibcpp
  hungarian
  )
add_compile_options(-g -rdynamic)
  

add_executable(semantic_slam
src/SLAM_No_persistence/SLAM_no_persistence.h
src/SLAM_Persistence/SLAM_persistence_simulation.h
src/SLAM_Headers/DataAssociation.h
src/SLAM_Headers/Marginalize.h
src/SLAM_Headers/LandmarkPoses.h
src/SLAM_Headers/LandmarkUpdates.h
# src/SLAM_Headers/LandmarkPosesExperiment.h
src/SLAM_Headers/Plotting.h
src/SLAM_Headers/Odometry.h
src/SLAM_Headers/Random.h
src/SLAM.cpp
#src/Data/Data/Graphing.cpp
)

include_directories(semantic_slam lib)
include_directories(semantic_slam 
/usr/local/Cellar/python@3.9/3.9.13_1/Frameworks/Python.framework/Versions/3.9/include/python3.9
/usr/local/lib/python3.9/site-packages/numpy/core/include)

target_link_libraries(
    semantic_slam
    ${SLAM_LIBS}
)
   # CMake build options
# Libraries:          # Boost          # ccache

###############################################################################
# Add components

# Build src
