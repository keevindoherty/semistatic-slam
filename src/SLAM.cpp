#include "SLAM_No_Persistence/SLAM_no_persistence.h"
#include "SLAM_Persistence/SLAM_persistence_simulation.h"
//#include "Data/Data/Graphing.cpp"

using namespace gtsam;
using namespace std;

#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>


void handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

int main(int argc, char** argv) {
  //signal(SIGSEGV, handler);   // install our handler
  fprintf(stderr, "Hi");
  AddGroundTruthLandmarkPersistence(Point2(3.0,3.0), Table);
  AddGroundTruthLandmarkPersistence(Point2(6.0,3.0), Table);
  AddGroundTruthLandmarkPersistence(Point2(7.0,4.0), Table);
  AddGroundTruthLandmarkPersistence(Point2(3.5,5.5), Table);
  AddGroundTruthLandmarkPersistence(Point2(10.0,2.0), Table);
  AddGroundTruthLandmarkPersistence(Point2(4.0,11.0), Table);
  AddGroundTruthLandmarkPersistence(Point2(13.0,12.0), Table);
  AddGroundTruthLandmarkPersistence(Point2(2.5,15.5), Table);
  NonlinearFactorGraph graph;
  plots p;
  OnePassPersistence(graph, p, landmarks_persistence, poses, observations, 0).print();
  OnePassPersistence(graph, p, landmarks_persistence, poses, observations, 1).print();
  OnePassPersistence(graph, p, landmarks_persistence, poses, observations, 2).print();
  OnePassPersistence(graph, p, landmarks_persistence, poses, observations, 3).print();
}

/*
TODOS:
- Fix QuatToEuler
*/

// #include "opencv2/opencv.hpp"
// #include <stdio.h>
// using namespace cv;
// int main(int argc, char** argv)
// {
//     if (argc != 2) {
//         printf("usage: DisplayImage.out <Image_Path>\n");
//         return -1;
//     }
//     Mat image;
//     image = imread(argv[1], 1);
//     if (!image.data) {
//         printf("No image data \n");
//         return -1;
//     }
//     namedWindow("Display Image", WINDOW_AUTOSIZE);
//     imshow("Display Image", image);
//     waitKey(0);
//     return 0;
// }