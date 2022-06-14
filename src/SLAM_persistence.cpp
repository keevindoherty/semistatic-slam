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
#include <persistence_filter/c++/include/persistence_filter_utils.h>



//Consider robot in a 5 by 5 grid
//Landmark coordinates at (2,0), (4,3), (3,5)
//Robot moves 1 space over and one space up in each step
//Assume the robot does 5 passes through the grid
//On pass 1, landmark one moves to (2,1)
//On pass 2, landmark two moves to (3,2)
//One pass 4, landmark three disappears

using namespace std;
using namespace gtsam;

const int ground_truth_num_landmarks = 3;
const int max_num_landmarks = 6;
const int num_poses = 6;

static Symbol x0('x',0), x1('x', 1), x2('x', 2), x3('x', 3), x4('x', 4), x5('x', 5);
static Symbol l1('l', 1), l2('l', 2), l3('l', 3), l4('l', 4), l5('l',5), l6('l',6);

static Point3 ground_truth_poses[num_poses] = {Vector3(0.0, 0.0, 0.0), Vector3(1.0, 1.0, 0.0), Vector3(2.0, 2.0, 0.0), Vector3(3.0, 3.0, 0.0), Vector3(4.0, 4.0, 0.0), Vector3(5.0, 5.0, 0.0)};

Point3 ground_truth_landmarks[ground_truth_num_landmarks] = {Vector3(2.0,0.0,0.0), Vector3(2.0,3.0,0.0), Vector3(3.0,5.0,0.0)};

static Symbol poses[num_poses] = {x0, x1, x2, x3, x4, x5};

static Symbol landmarks[max_num_landmarks] = {l1, l2, l3, l4, l5, l6};

static Vector3 odometry(1.0, 1.0, 0.0);

static Vector2 mNoise(0.01, 0.02);

static Vector3 oNoise(0.02, 0.02, 0.01);

static double lambda_u = 1;
static double lambda_l = .01;

//P_M: Probability of missed detection (object exists, but is not detected)
static double P_M = 0.01;
//P_F: Probability of false positive (object doesn't exist, but is detected)
static double P_F = 0.01;
//time counter for the persistence filter
double t = 1.0;

std::function<double(double)> logS_T = std::bind(log_general_purpose_survival_function, std::placeholders::_1, lambda_l, lambda_u);

//Array of persistence filters for the number of landmarks
PersistenceFilter landmark_filters[max_num_landmarks] = {PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T)};
//Map with feature locations as keys and symbol indices as values
map<int, Vector3> index_to_feature_map;
//stores current symbol_index
int current_landmark_index = ground_truth_num_landmarks;
int current_pose_index = 0;

//This method adds noise to the provided measurement to mimic a real life measurement with noisy equipment
double add_noise(double stddev, double measurement){
    double mean = 0.0;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    // Add Gaussian noise
    return measurement+dist(generator);
}

//This method increments all the time stamps so the persistence filter can behave accuratel

//This method gives the distance between two vectors when they are provided
double distance(Vector3 v1, Vector3 v2){
  double distance = sqrt(pow(v1.x()-v2.x(),2.0)+pow(v1.y()-v2.y(),2.0));
  return distance;
}

//This method determines the index of a detected feature
int feature_to_index(Vector3 detected_feature){
  for (auto element: index_to_feature_map){
     if (distance(element.second, detected_feature)<0.1){
       return element.first;
     }
  }
  index_to_feature_map[current_landmark_index]= detected_feature;
  current_landmark_index++;
  return current_landmark_index-1;
}


Vector3 index_to_feature(int symbol_no){
  /*Returns a feature (vector) given a symbol number*/
  return index_to_feature_map[symbol_no];
}

bool ground_truth(Vector3 v1){
  /*This method determines if a point is a ground truth landmark 
    Assumes any landmark within 0.1 of the ground truth landmark is a ground truth landmark
  */
  for (int i =0; i < ground_truth_num_landmarks; i++){
    if (distance(ground_truth_landmarks[i], v1)<0.1){
      return true;
    }
  }
  return false;
}

//This method applies the persistence filter to a point that is in the ground truth
bool persistence(int landmark_no){
  //Landmark no: Indicates landmark 1, landmark 2, landmark 3, etc
  if (ground_truth(index_to_feature(landmark_no))){
    double random = rand()%100;
    //fprintf(stderr, "Random: %f\n", random);
    bool false_positive = false;
    bool false_negative = false;
    if (random < 100*P_M){
      false_positive = true;
    }
    if (random > 100*(1-P_F)){
      false_negative = true;
    }
    if (false_negative || false_positive){
      landmark_filters[landmark_no].update(false, t, P_M, P_F);
    }
    else if (!false_positive){
      //fprintf(stderr, "persistence, landmark number: %d", landmark_no);
      landmark_filters[landmark_no].update(true, t, P_M, P_F);
    }
    else if (false_positive){
      //TODO: Fill in with what happens if a false positive is triggered
    }
  }
  else{
    landmark_filters[landmark_no].update(false, t, P_M, P_F);
  }
  if (landmark_filters[landmark_no].predict(t) > 0.75){
    //return true;
  }
  return true;
}


// void update_graph_landmark(NonlinearFactorGraph graph, int i, Vector3 pose, Vector3 landmark){
//   auto measurementNoise = noiseModel::Diagonal::Sigmas(
//       mNoise);
//   /* Adds landmark to the graph */
//   double angle = atan((landmark.y()-pose.y())/(landmark.x()-pose.x()));
//   Rot2 bearing = Rot2::fromAngle(add_noise(mNoise.y(), angle));
//   bearing.print();
//   double range = sqrt(pow(landmark.y()-pose.y(), 2.0)+pow(landmark.x()-pose.x(), 2.0));
//   double noisy_range = add_noise(mNoise.x(), range);
//   graph.emplace_shared <BearingRangeFactor<Pose2, Point2>> (poses[i], landmarks[feature_to_index(landmark)], bearing, noisy_range, measurementNoise);
// }


void one_pass(NonlinearFactorGraph graph) {
  auto odometryNoise = noiseModel::Diagonal::Sigmas(
      oNoise);
   
  auto measurementNoise = noiseModel::Diagonal::Sigmas(
      mNoise);
  // Adding prior
  for (int i = 0; i < 5; i++){
    //Add a factor between the two poses
    Pose2 noisy_odometry = Pose2(add_noise(oNoise.x(), odometry.x()), add_noise(oNoise.y(), odometry.y()), add_noise(oNoise.z(), odometry.z()));
    graph.emplace_shared<BetweenFactor<Pose2> >(poses[i], poses[i+1], noisy_odometry, odometryNoise);
    int map_size = index_to_feature_map.size();
    for (int j = 0; j <map_size; j++){
      bool p = persistence(j);
      fprintf(stderr, "p: %d, i: %d,j: %d\n", p, i, j);
      Vector3 feat = index_to_feature(j);
      if (feat.x() >= ground_truth_poses[i].x() && p){
        double angle = atan((feat.y()-ground_truth_poses[i].y())/(feat.x()-ground_truth_poses[i].x()));
        Rot2 bearing = Rot2::fromAngle(add_noise(mNoise.y(), angle));
        bearing.print();
        double range = sqrt(pow(feat.y()-ground_truth_poses[i].y(), 2.0)+pow(feat.x()-ground_truth_poses[i].x(), 2.0));
        double noisy_range = add_noise(mNoise.x(), range);
        fprintf(stderr, "Adding factor: i: %d,j: %d\n", i, j);
        graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(poses[i], landmarks[j], bearing, noisy_range, measurementNoise);
      }
    }
    t+=1.0;
  }
  Values initialEstimate;

  for (int i = 0; i < 6; i++){
      initialEstimate.insert(poses[i], Pose2(rand()%500/100.0, rand()%500/100.0, rand()%500/100.0));
  }

   for (int i = 0; i < 3; i++){
      initialEstimate.insert(landmarks[i], Point2(rand()%500/100.0, rand()%500/100.0));
  }
  //graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(x0, l1, -0.0247947181, 1.98760264, measurementNoise);
  graph.print();
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final: ");
  //result.at<Point2>(l1);
}

int main(int argc, char** argv) {
    // Create the keys we need
  NonlinearFactorGraph graph;
  Vector3 pNoise(0.0,0.0,0.0);
  auto priorNoise = noiseModel::Diagonal::Sigmas(
    pNoise);
  Pose2 prior(0.0, 0.0, 0.0);     
  graph.addPrior(poses[0], prior, priorNoise);  // add directly to graph
  
  for (int i = 0; i < ground_truth_num_landmarks; i ++){
    index_to_feature_map[i] = ground_truth_landmarks[i];
  }
   one_pass(graph);
  // one_pass(graph);
  // ground_truth_landmarks[0] = Vector3(2.0,1.0,0.0);
  // one_pass(graph);

}

