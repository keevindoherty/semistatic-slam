cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(planar_segmentation)

set(Qt5_DIR /usr/local/Cellar/qt@5/5.15.8_1/lib/cmake/Qt5)
set(VTK_DIR /usr/local/Cellar/vtk/9.2.5_1/lib/cmake/vtk-9.2)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (planar_segmentation planesegmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})