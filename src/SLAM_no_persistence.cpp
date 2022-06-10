// As this is a planar SLAM example, we will use Pose2 variables (x, y, theta) to represent
// the robot positions and Point2 variables (x, y) to represent the landmark coordinates.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <persistence_filter/c++/include/persistence_filter.h>

//Consider robot in a 5 by 5 grid
//Landmark coordinates at (2,0), (4,3), (3,5)
//Robot moves 1 space over and one space up in each step
//Assume the robot does 5 passes through the grid
//On pass 1,

using namespace std;
using namespace gtsam;

PersistenceFilter filt();

static Symbol x0('x',0), x1('x', 1), x2('x', 2), x3('x', 3), x4('x', 4), x5('x', 5);
static Symbol l1('l', 1), l2('l', 2), l3('l', 3);

Vector3 ground_truth_poses[6] = {Vector3(0.0, 0.0, 0.0), Vector3(1.0, 1.0, 0.0), Vector3(2.0, 2.0, 0.0), Vector3(3.0, 3.0, 0.0), Vector3(4.0, 4.0, 0.0), Vector3(5.0, 5.0, 0.0)};

Vector3 ground_truth_landmarks[3] = {Vector3(2.0,0.0,0.0), Vector3(2.0,3.0,0.0), Vector3(3.0,5.0,0.0)};

Symbol poses[6] = {x0, x1, x2, x3, x4, x5};

Symbol landmarks[3] = {l1, l2, l3};

Vector3 odometry(1.0, 1.0, 0.0);

double lambda_u = 1;
double lambda_l = .01;

double P_M = .2;
double P_F = .01;

double add_noise(double stddev, double measurement){
    double mean = 0.0;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    // Add Gaussian noise
    return measurement+dist(generator);
}

void one_pass() {
  NonlinearFactorGraph graph;

  Vector2 mNoise = Vector2(0.01, 0.02);

  Vector3 pNoise = Vector3(0.0,0.0,0.0);

  Vector3 oNoise = Vector3(0.02, 0.02, 0.01);

  auto odometryNoise = noiseModel::Diagonal::Sigmas(
      oNoise);
  
  auto priorNoise = noiseModel::Diagonal::Sigmas(
      pNoise);
  
  auto measurementNoise = noiseModel::Diagonal::Sigmas(
      mNoise);
  // Adding prior
  Pose2 prior(0.0, 0.0, 0.0);
         
  graph.addPrior(poses[0], prior, priorNoise);  // add directly to graph


  for (int i = 0; i < 5; i++){
    //Add a factor between the two poses
    Pose2 noisy_odometry = Pose2(add_noise(oNoise.x(), odometry.x()), add_noise(oNoise.y(), odometry.y()), add_noise(oNoise.z(), odometry.z()));
    graph.emplace_shared<BetweenFactor<Pose2> >(poses[i], poses[i+1], noisy_odometry, odometryNoise);
    //Bearing between pose and landmark in radians
    for (int j = 0; j <3; j++){
      if (ground_truth_landmarks[j].x()>= ground_truth_poses[i].x()){
        double angle = atan((ground_truth_landmarks[j].y()-ground_truth_poses[i].y())/(ground_truth_landmarks[j].x()-ground_truth_poses[i].x()));
        Rot2 bearing = Rot2::fromAngle(add_noise(mNoise.y(), angle));
        bearing.print();
        double range = sqrt(pow(ground_truth_landmarks[j].y()-ground_truth_poses[i].y(), 2.0)+pow(ground_truth_landmarks[j].x()-ground_truth_poses[i].x(), 2.0));
        double noisy_range = add_noise(mNoise.x(), range);
        graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(poses[i], landmarks[j], bearing, noisy_range, measurementNoise);
      }
    }
  }

  Values initialEstimate;

  for (int i = 0; i < 6; i++){
      initialEstimate.insert(poses[i], Pose2(rand()%500/100.0, rand()%500/100.0, rand()%500/100.0));
  }

   for (int i = 0; i < 3; i++){
      initialEstimate.insert(landmarks[i], Point2(rand()%500/100.0, rand()%500/100.0));
  }

  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  //result.at<Point2>(l1);
  result.print("Final Result:\n");
}

int main(int argc, char** argv) {
    // Create the keys we need

  one_pass();

}
