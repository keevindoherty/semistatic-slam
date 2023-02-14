# cmake version and project name
cmake_minimum_required( VERSION 3.15 )

project( pointcloudprocessing )

set(CMAKE_CXX_STANDARD 17)

set(Qt5_DIR /usr/local/Cellar/qt@5/5.15.8_1/lib/cmake/Qt5)
set(VTK_DIR /usr/local/Cellar/vtk/9.2.5_1/lib/cmake/vtk-9.2)

# Set up opencv Library
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Set up pcl Library 
find_package( PCL 1.3 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS} )

# Compiling and linking
add_executable( rgbd2pcd rgbd2pcd.cpp )
add_executable( planesegmentation pcl/planesegmentation.cpp )
target_link_libraries( rgbd2pcd ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries( planesegmentation ${OpenCV_LIBS} ${PCL_LIBRARIES})
