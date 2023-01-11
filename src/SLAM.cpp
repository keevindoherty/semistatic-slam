#include "SLAM_No_Persistence/SLAM_no_persistence.h"
#include "SLAM_Persistence/SLAM_persistence_simulation.h"
//#include "Data/Data/Graphing.cpp"

using namespace gtsam;
using namespace std;

int main(int argc, char** argv) {
  AddGroundTruthLandmark(Point2(15.0,15.0), Table);
  AddGroundTruthLandmark(Point2(6.0,3.0), Table);
  AddGroundTruthLandmark(Point2(7.0,4.0), Table);
  AddGroundTruthLandmark(Point2(3.5,5.5), Table);
  AddGroundTruthLandmark(Point2(10.0,2.0), Table);
  AddGroundTruthLandmark(Point2(4.0,11.0), Table);
  AddGroundTruthLandmark(Point2(13.0,12.0), Table);
  AddGroundTruthLandmark(Point2(2.5,15.5), Table);
  NonlinearFactorGraph graph;
  plots p;
  OnePass(graph, p, landmarks, poses, observations, 0);
  OnePass(graph, p, landmarks, poses, observations, 1);
  OnePass(graph, p, landmarks, poses, observations, 2);
  OnePass(graph, p, landmarks, poses, observations, 3).print();
}