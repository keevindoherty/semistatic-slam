cmake_minimum_required(VERSION 3.0)
project(SLAM_no_persistence)

# new feature to Cmake Version > 2.8.12
# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

add_subdirectory(lib/gtsam)
add_subdirectory(lib/persistence_filter)

find_package(GTSAM 4.1 REQUIRED)
#find_package(PersistenceFilter REQUIRED)

set(SLAM_LIBS
  persistence_filter
  gtsam
  )

add_executable(SLAM_no_persistence src/SLAM_no_persistence.cpp)
include_directories(SLAM_no_persistence lib)

target_link_libraries(
    SLAM_no_persistence
    ${SLAM_LIBS}
)
   # CMake build options
# Libraries:          # Boost          # ccache

###############################################################################
# Add components

# Build src