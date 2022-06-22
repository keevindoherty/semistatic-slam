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
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <persistence_filter/c++/include/persistence_filter.h>
#include <persistence_filter/c++/include/persistence_filter_utils.h>



/*To change the positions or ground_truth_landmarks:
    - Change ground_truth_num_landmarks
    - Change num_poses
    - Changed ground_truth_poses
    - Change ground_truth_landmarks
    - Change initial_estimate_poses
    - Change initial_estimate_landmarks
*/

using namespace std;
using namespace gtsam;

bool debug = false;

const int ground_truth_num_landmarks = 3;
const int max_num_landmarks = 15;
const int num_poses = 7;

static Point3 ground_truth_poses[num_poses] = {Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0), Vector3(2.0, 2.0, 0.0), Vector3(3.0, 3.0, 0.0), Vector3(4.0, 4.0, 0.0), Vector3(5.0, 5.0, 0.0), Vector3(0.0,0.0,0.0)};

Point3 ground_truth_landmarks[ground_truth_num_landmarks] = {Vector3(2.0,1.0,0.0), Vector3(2.0,3.0,0.0), Vector3(3.0,5.0,0.0)};

NonlinearFactorGraph graph;

static Vector3 odometry(1.0, 1.0, 0.0);

static Vector3 pNoise(0.0,0.0,0.0);

static Vector2 mNoise(0.01, 0.02);

static Vector3 oNoise(0.02, 0.02, 0.01);

//Symbol hashed using h function defined below to get
map<string,bool> removed_landmarks = {};

static double lambda_u = 1;
static double lambda_l = .01;

//P_M: Probability of missed detection (object exists, but is not detected)
static double P_M = 0.15;
//P_F: Probability of false positive (object doesn't exist, but is detected)
static double P_F = 0.1;
//time counter for the persistence filter
double t[max_num_landmarks];

int passes = 0;

std::uniform_real_distribution<double> distribution(0.0, 5.0);
static std::default_random_engine generator(time(0));

std::function<double(double)> logS_T = std::bind(log_general_purpose_survival_function, std::placeholders::_1, lambda_l, lambda_u);

//Array of persistence filters for the number of landmarks
PersistenceFilter landmark_filters[max_num_landmarks] = {
  PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), 
  PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), 
  PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T)
};
//Map with feature locations as keys and symbol indices as values
map<int, Vector3> index_to_feature_map;
//stores current symbol_index
int current_landmark_number = ground_truth_num_landmarks;
int current_pose_index = 0;

int past_pose_index = 0;
int past_landmark_number = 0;

void initializeTimes(){
  for (int i = 0; i < max_num_landmarks; i++){
    t[i]=1.0;
  }
}

string h(Symbol s){
  /*This method hashes a vector to use it as a key in the map*/
  return std::string(s);
  //s.chr()+s.index();
}

void initializeRemovedLandmarks(){
  for(int i = 0; i < max_num_landmarks; i++){
    removed_landmarks[h(Symbol('l',i))]=true;
  }  
}

//This method adds noise to the provided measurement to mimic a real life measurement with noisy equipment
double add_noise(double stddev, double measurement){
    double mean = 0.0;
    std::normal_distribution<double> dist(mean, stddev);

    // Add Gaussian noise
    return measurement+dist(generator);
}

//This method gives the distance between two vectors when they are provided
double distance(Vector3 v1, Vector3 v2){
  double distance = sqrt(pow(v1.x()-v2.x(),2.0)+pow(v1.y()-v2.y(),2.0));
  return distance;
}

void changeGroundTruthLandmark(Vector3 pos, int removedLandmark){
  ground_truth_landmarks[removedLandmark] = pos;
  index_to_feature_map[current_landmark_number] = pos;
  current_landmark_number++;
}

//This method determines the index of a detected feature
int feature_to_index(Vector3 detected_feature){
  for (auto element: index_to_feature_map){
     if (distance(element.second, detected_feature)<0.1){
       return element.first;
     }
  }
  index_to_feature_map[current_landmark_number]= detected_feature;
  current_landmark_number++;
  return current_landmark_number-1;
}

void removeFactors(const set<size_t>& deleteFactors, gtsam::NonlinearFactorGraph& graph){
  auto itr = deleteFactors.begin();
  for (int i = 0; i < deleteFactors.size(); i++){
     graph.remove(*itr);
     itr++;
  }
}

bool keyInKeyVector(const KeyVector& keys, Key k){
  //Returns if a key is in the key vector
  for (auto key: keys){
    Symbol(key).print();
    if (k == key){
      return true;
    }
  }
  return false;
}

void eraseKeys(const KeyVector& keys, gtsam::NonlinearFactorGraph& graph) {
  //Modifies graph by erasing keys and any factors connected to the keys
  auto it = graph.begin();
  int size = graph.size();
  for (auto key: keys){
    int num = 0;
    for (auto factor: graph){
      bool a = keyInKeyVector(factor->keys(), key);
      if(a){
        graph.erase(it+num);
      }
      //TODO: Why is this necessary?
      if (num == graph.size()-1){
        break;
      }
      num++;
    }
  }
}

void addFactors(const gtsam::GaussianFactorGraph& newFactors, gtsam::NonlinearFactorGraph& graph) {
  for(const auto& f: newFactors){
    gtsam::LinearContainerFactor linearContainer(f);
    gtsam::NonlinearFactorGraph fg = linearContainer.ConvertLinearGraph(newFactors);
    for(const auto& nlfactor: fg){
    graph.add(nlfactor);
    }
    break;
  }
  graph.print("Gaussian Graph");
}

gtsam::KeyVector all_but_one(Key key, const gtsam::GaussianFactorGraph& graph){
  auto it = graph.begin();
  set<Key> keys;
  for (auto factor: graph){
    for(auto k: factor->keys()){
      if(k!=key){
         keys.insert(k);
      }
    }
  }
  gtsam::KeyVector ret;
  for(Key key: keys){
    ret.push_back(key);
  }
  return ret;
}

void marginalize(const gtsam::Key &key, gtsam::NonlinearFactorGraph& graph, const gtsam::Values &linearization_point) {

  // Marginalizes specified keys from a nonlinear factor graph by computing the subgraph containing the marginalized keys
  // and their attached factors, linearizing this graph, applying marginalization upon the linearized graph, and adding the 
  // resulting factors back into the original graph.

  // Determine the factors touching `key` and put them in a separate graph
  set<size_t> removedFactorSlots;
  const VariableIndex variableIndex(graph);
  const auto& slots = variableIndex[key];
  removedFactorSlots.insert(slots.begin(), slots.end());

  NonlinearFactorGraph removedFactors;
  for(size_t slot: removedFactorSlots) {
    if (graph.at(slot)) {
      removedFactors.add(graph.at(slot));
    }
  }

  // Linearize the graph of factors touching the key
  KeyVector k = {key};
  auto g_ptr = removedFactors.linearize(linearization_point);
  GaussianFactorGraph g = *g_ptr;

   //Marginalize this graph
  gtsam::GaussianFactorGraph m = *(g.marginal(all_but_one(key,g)));

  // //Remove the marginalized keys from the original graph
  eraseKeys(k, graph);

  //Add induced factors to the graph
  addFactors(m, graph);
}


Vector3 index_to_feature(int landmark_no){
  //Returns a feature (vector) given a symbol number
  return index_to_feature_map[landmark_no];
}

bool ground_truth(Vector3 v1){
  // This method determines if a point is a ground truth landmark 
  // Assumes any landmark within 0.1 of the ground truth landmark is a ground truth landmark
  for (int i =0; i < ground_truth_num_landmarks; i++){
    if (distance(ground_truth_landmarks[i], v1)<0.1){
      return true;
    }
  }
  return false;
}

bool persistence(int landmark_no){
  fprintf(stderr, "Landmark no = %d, time = %f\n", landmark_no, t[landmark_no]);
  //Landmark no: Indicates landmark 1, landmark 2, landmark 3, etc
  if (ground_truth(index_to_feature(landmark_no))){
    //Only simulates false positives/false negatives if an object is in the ground truth
    //Uses an RNG to simulate false positives and false negatives
    double random = distribution(generator)*20.0;
    bool false_positive = false;
    bool false_negative = false;
    if (random < 100*P_F){
      fprintf(stderr, "False positive\n");
      false_positive = true;
    }
    if (random > 100*(1-P_M)){
      fprintf(stderr, "False negative\n");
      false_negative = true;
    }
    if (false_negative || false_positive){
      landmark_filters[landmark_no].update(false, t[landmark_no], P_M, P_F);
    }
    else{
      //If a false positive/negative aren't being simulated, then the landmark is marked as true
      //fprintf(stderr, "Updating with true\n");
      landmark_filters[landmark_no].update(true, t[landmark_no], P_M, P_F);
      //fprintf(stderr, "Updated\n");
    }
    if (false_positive){
      //fprintf(stderr, "Line 1\n");
      index_to_feature_map[current_landmark_number] = Vector3(distribution(generator),distribution(generator),distribution(generator));
      current_landmark_number++;
      //fprintf(stderr, "Line 2. Increasing time.\n");
      t[landmark_no]++;
      return persistence(current_landmark_number-1);
    }
    //fprintf(stderr, "Increasing time.\n");
    t[landmark_no]++;
  }
  else{
    //Triggered in the case of a false positive
    //fprintf(stderr, "triggered\n");
    landmark_filters[landmark_no].update(false, t[landmark_no], P_M, P_F);
    //fprintf(stderr, "updated filter\n");
    t[landmark_no]++;
  }
  if (landmark_filters[landmark_no].predict(t[landmark_no]) > 0.75){
    return true;
  }
  return false;
}

Values one_pass(Values estimate) {
  initializeRemovedLandmarks();
  fprintf(stderr, "\n\n PASS %d: \n", passes);
  //Noise models
  auto priorNoise = noiseModel::Diagonal::Sigmas(
    pNoise);

  Pose2 prior(0.0, 0.0, 0.0);  

  auto odometryNoise = noiseModel::Diagonal::Sigmas(
      oNoise);
   
  auto measurementNoise = noiseModel::Diagonal::Sigmas(
      mNoise);

  // Adding prior only on first pass
  if (passes == 0){
     graph.addPrior(Symbol('x',current_pose_index), prior, priorNoise);  // add directly to graph
  }

  for (int i = 0; i < num_poses-1; i++){
    //Add a factor between the two poses
    Vector3 odometry(ground_truth_poses[i+1].x()-ground_truth_poses[i].x(), ground_truth_poses[i+1].y()-ground_truth_poses[i].y(), ground_truth_poses[i+1].z()-ground_truth_poses[i].z());
    //Add noise to ground truth odometry to simulate real life noise
    Pose2 noisy_odometry = Pose2(add_noise(oNoise.x(), odometry.x()), add_noise(oNoise.y(), odometry.y()), add_noise(oNoise.z(), odometry.z()));
    //Factor between two consecutive poses
    graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x',current_pose_index), Symbol('x',current_pose_index+1), noisy_odometry, odometryNoise);

    int map_size = index_to_feature_map.size();
    for (int landmark_no = 0; landmark_no <map_size; landmark_no++){
      bool p = persistence(landmark_no);
      Vector3 feat = index_to_feature(landmark_no);
      fprintf(stderr, "Landmark %d, position %d\n", landmark_no, i);
      //Feature is in robot's line of sight (has greater x coordinate than robot) and persistence is true
      if (feat.x() >= ground_truth_poses[i].x() && p){
        double angle = atan((feat.y()-ground_truth_poses[i].y())/(feat.x()-ground_truth_poses[i].x()));
        Rot2 bearing = Rot2::fromAngle(add_noise(mNoise.y(), angle));
        double range = sqrt(pow(feat.y()-ground_truth_poses[i].y(), 2.0)+pow(feat.x()-ground_truth_poses[i].x(), 2.0));
        double noisy_range = add_noise(mNoise.x(), range);
        graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(Symbol('x',current_pose_index), Symbol('l',landmark_no), bearing, noisy_range, measurementNoise);
      }
    }
    current_pose_index++;
  }
  fprintf(stderr, "Constructing estimate\n");
  Values currentEstimate;
  if(passes == 0){
    past_pose_index = current_pose_index;
    past_landmark_number = current_landmark_number;
  }

  //Update removed_landmarks to reflect if any landmarks have been removed
  for (Key key:graph.keys()){
      removed_landmarks[h(key)] = false;
  }
  

  for (int i = 0; i <= current_pose_index; i++){
     currentEstimate.insert(Symbol('x',i), estimate.at<Pose2>(Symbol('x', i% past_pose_index)));
  }

  for (int i = 0; i < current_landmark_number; i++){
     if(removed_landmarks[h(Symbol('l',i))]){
      continue;
     }
     if(i >= past_landmark_number){
       currentEstimate.insert(Symbol('l',i), Point2(distribution(generator), distribution(generator)));
     }
     else{
       currentEstimate.insert(Symbol('l',i), estimate.at<Point2>(Symbol('l', i)));
     }
  }

  past_pose_index = current_pose_index;
  past_landmark_number = current_landmark_number;
  
  graph.print("Graph: ");

  LevenbergMarquardtOptimizer optimizer(graph, currentEstimate);
  Values new_result = optimizer.optimize();
  new_result.print("Final: ");
  passes++;

  return new_result;

}


int main(int argc, char** argv) {
    // Create the keys we need
  initializeTimes();

  for (int i = 0; i < ground_truth_num_landmarks; i ++){
    index_to_feature_map[i] = ground_truth_landmarks[i];
  }
  Values initialEstimate;

  initialEstimate.insert(Symbol('x',0), Pose2(4.07, 2.49, 0.73));
  initialEstimate.insert(Symbol('x',1), Pose2(1.58, 4.3, 2.72));
  initialEstimate.insert(Symbol('x',2), Pose2(0.44, 3.78, -2.05319));
  initialEstimate.insert(Symbol('x',3), Pose2(2.09, 4.4, 1.65));
  initialEstimate.insert(Symbol('x',4), Pose2(4.92, 0.42, -1.41319));
  initialEstimate.insert(Symbol('x',5), Pose2(0.03, 3.27, 2.29));
  initialEstimate.insert(Symbol('x',6), Pose2(2.09, 4.4, 1.65));

  initialEstimate.insert(Symbol('l',0), Point2(3.4, 1.12)); 
  initialEstimate.insert(Symbol('l',1), Point2(3.03, 1.69));
  initialEstimate.insert(Symbol('l',2), Point2(2.09, 1.57));

  Values current_result = one_pass(initialEstimate); 
  
  changeGroundTruthLandmark(Vector3(4.0,2.0,0.0), 1);

  current_result = one_pass(current_result);
  marginalize(Symbol('l',1), graph, current_result);
  current_result = one_pass(current_result);

}
