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
#include <math.h>
#include "matplotlibcpp.h"

// Next Step: Data association (Record observations from Ground Truth)
// Associate with existing landmarks and update accordingly
// Update the rest w false observation

//Initialization time persistence filter

using namespace std;
using namespace gtsam;
using namespace matplotlibcpp;

enum SemanticType{
  Book,
  Chair,
  Table
};

class RobotPose{
    public:
    gtsam::Symbol symbol;
    gtsam::Pose2 position;
    //category indicates if position is a landmark or pose

    RobotPose(const Symbol& s, const Pose2& pos) {
       symbol = s;
       position = pos;
    }
     void print(){
      fprintf(stderr, "Robot Pose. Symbol: Symbol('l',%d), position: (%f, %f)\n", symbol.index(), position.x(), position.y());
    }
};

class Landmark{
   public:
    gtsam::Symbol symbol;
    gtsam::Point2 position;
    SemanticType semanticType;
    //category indicates if position is a landmark or pose
    PersistenceFilter filter{logS_T};
    double t = 1.0;

    Landmark(const Symbol& s, const Point2& pos, const SemanticType& sem) {
       symbol = s;
       position = pos;
       semanticType = sem;
    }

    void print(){
      fprintf(stderr, "Landmark. Symbol: Symbol('l',%d), position: (%f, %f)\n", symbol.index(), position.x(), position.y());
    }
    static double lambda_u;
    static double lambda_l;
    static std::function<double(double)> logS_T;
};
std::function<double(double)> Landmark::logS_T = std::bind(log_general_purpose_survival_function, std::placeholders::_1, Landmark::lambda_l, Landmark::lambda_u);
double Landmark::lambda_u = 1;
double Landmark::lambda_l = 0.01;

//GLOBAL VARIABLES
double pi = atan(1)*4;
double random_seed = 11.0;
std::uniform_real_distribution<double> distribution(0.0, 5.0);
static std::default_random_engine generator(random_seed);
//Vectors of class Position for landmarks and poses
vector<Landmark> landmarks;
vector<RobotPose> poses;
//Number of poses (per pass) and number of ground truth landmarks
int num_poses = 7;
int num_ground_truth_landmarks = 0;
//Ground truth landmarks
vector<Landmark> ground_truth_landmarks;
//Given in x-y-z coordinates
vector <Vector3> ground_truth_poses = {Vector3(0.0,0.0,0.0), Vector3(1.0,1.0,0.0), Vector3(2.0,2.0,0.0), Vector3(3.0,3.0,0.0), Vector3(4.0,4.0,0.0), Vector3(5.0,5.0,0.0), Vector3(0.0,0.0,0.0)};
//Noise models and constants
//QUESTION: Where should these go?
static Vector3 pNoise(0.0,0.0,0.0);
static Vector2 mNoise(0.01, 0.02);
static Vector3 oNoise(0.02, 0.02, 0.01);

const auto priorNoise = noiseModel::Diagonal::Sigmas(
    pNoise);
const auto odometryNoise = noiseModel::Diagonal::Sigmas(
    oNoise);   
const auto measurementNoise = noiseModel::Diagonal::Sigmas(
    mNoise);

//P_M: Probability of missed detection (object exists, but is not detected)
static double P_M = 0.05;
//P_F: Probability of false positive (object doesn't exist, but is detected)
static double P_F = 0.05;
//Vector of recently updated ground truth symbols (ie landmarks with this symbol are currently set to ground truth position)
//This vector is cleared every iteration, and landmarks from this iteration are updated

double AddNoise(double stddev, double measurement){
    double mean = 0.0;
    std::normal_distribution<double> dist(mean, stddev);

    // Add Gaussian noise
    return measurement+dist(generator);
}

void AddGroundTruthLandmark(const Point2& pos, const SemanticType& sem){
    Landmark p(Symbol('l',landmarks.size()), pos, sem);
    landmarks.push_back(p);
    ground_truth_landmarks.push_back(p);
}

void MoveGroundTruthLandmark(Vector3 new_pos, int landmark_no){
    fprintf(stderr, "MoveGroundTruthLandmark not yet defined\n");
}

double Distance(Point2 v1, Point2 v2){
  return sqrt(pow(v1.x()-v2.x(),2.0)+pow(v1.y()-v2.y(),2.0));
}

bool GroundTruth(Point2 v){
    //Returns if vector is in ground truth
    for (Landmark l: ground_truth_landmarks){
        if(Distance(l.position, v)<0.5){
            return true;
        }
    }
    return false;
}
//RENAME
Pose2 Odometry(Pose2 p1, Vector3 v2){
//Gives the robots odometry given its starting position and ending coordinates
//fprintf(stderr, "Pose passed in: (%f, %f, %f)\n", p1.x(), p1.y(), p1.theta());
//fprintf(stderr, "Vector passed in: (%f, %f, %f)\n", v2.x(), v2.y(), v2.z());
double orig_angle = atan((v2.y()-p1.y())/(v2.x()-p1.x()));
if(v2.x() < p1.x()){ orig_angle = pi+orig_angle; }
double r = sqrt(pow(v2.x()-p1.x(),2)+pow(v2.y()-p1.y(),2));
double x = r*cos(orig_angle-p1.theta());
double y = r*sin(orig_angle-p1.theta());
fprintf(stderr, "x = %f, y = %f\n", x, y);
double theta = atan(y/x);
if(x < 0){ theta= pi+theta; }
//fprintf(stderr, "Odometry: (%f, %f, %f)\n", x, y, theta);
Pose2 odometry(x, y, theta);
return odometry;
}

int Persistence(int landmark_no){
  //Landmark no: Indicates landmark 1, landmark 2, landmark 3, etc
  //Outputs: 1 indicates a fase positive induced by ground truth landmark
  //2 indicates a false positive landmark_no passed into persistence
  //3 indicates false negative
  //0 indicates none of the above
  if (GroundTruth(landmarks[landmark_no].position)){
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
      fprintf(stderr, "Updating landmark %d with false\n", landmark_no);
      landmarks[landmark_no].filter.update(false, landmarks[landmark_no].t, P_M, P_F);
      //fprintf(stderr, "Time updated for landmark %d\n", landmark_no);
      landmarks[landmark_no].t++;
    }
    else{
      //If a false positive/negative aren't being simulated, then the landmark is marked as true
      fprintf(stderr, "Updating landmark %d with true\n", landmark_no);
      landmarks[landmark_no].filter.update(true, landmarks[landmark_no].t, P_M, P_F);
      //fprintf(stderr, "Time updated for landmark %d\n", landmark_no);
      landmarks[landmark_no].t++;
    }
    if (false_positive){
      //Add a false positive landmark to landmark list
      landmarks.push_back(Landmark(Symbol('l',landmarks.size()), Point2(distribution(generator),distribution(generator)), Book));
      fprintf(stderr, "Updating landmark %d with true\n", landmark_no);
      landmarks[landmarks.size()-1].filter.update(true, landmarks[landmarks.size()-1].t, P_M, P_F);
      landmarks[landmarks.size()-1].t++;
      //fprintf(stderr, "Time updated for landmark %d\n", landmarks.size()-1);
    }
    if (false_positive){ return 1; }
    if(false_negative){ return 3; }
  //Landmark in the ground truth, not false positive or negative
  return 0;
  }
  else{
    //Running persistence on a previous false positive
    landmarks[landmark_no].filter.update(false, landmarks[landmark_no].t, P_M, P_F);
    landmarks[landmark_no].t++;
    fprintf(stderr, "Running persistence on a false positive. Updating landmark %d with false\n", landmark_no);
    return 2;
  }
}

Pose2 estimate_from_odom(Pose2 past_pos, Pose2 odometry){
  //fprintf(stderr, "Past position: (%f, %f, %f)\n", past_pos.x(), past_pos.y(), past_pos.theta());
  //fprintf(stderr, "odometry: (%f, %f, %f)\n", odometry.x(), odometry.y(), odometry.theta());
  double r = sqrt(pow(odometry.x(),2)+pow(odometry.y(),2));
  double x = past_pos.x()+r*cos(past_pos.theta()+odometry.theta());
  double y = past_pos.y()+r*sin(past_pos.theta()+odometry.theta());
  double theta = past_pos.theta()+odometry.theta();
  Pose2 estimate(x, y, theta);
  //fprintf(stderr, "Estimate: (%f, %f, %f)\n", estimate.x(), estimate.y(), estimate.theta());
   return estimate;
}

Point2 estimate_from_range(Pose2 past_pos, double range, double bearing){
  //fprintf(stderr, "Past position: (%f, %f, %f)\n", past_pos.x(), past_pos.y(), past_pos.theta());
  double r = range;
  double x = past_pos.x()+r*cos(past_pos.theta()+bearing);
  double y = past_pos.y()+r*sin(past_pos.theta()+bearing);
  double theta = past_pos.theta()+bearing;
  Point2 estimate(x, y);
  //fprintf(stderr, "Landmark estimate: (%f, %f)\n", estimate.x(), estimate.y());
   return estimate;
}

Point2 addBearingRangeFactor(int pose_no, int landmark_no, NonlinearFactorGraph& graph){
  //Returns approximate location of landmark from measurement (noisy range and bearing)
   //fprintf(stderr, "Position: %d, Landmark: %d\n", pose_no, landmark_no);
   Point2 feat = ground_truth_landmarks[landmark_no].position;
   Pose2 p = poses[pose_no].position;
   fprintf(sdterr, "Hello\n");  
   double orig_angle = atan((feat.y()-p.y())/(feat.x()-p.x()));
   if(feat.x() < p.x()){ orig_angle = pi+orig_angle; }
   double range = sqrt(pow(feat.x()-p.x(),2)+pow(feat.y()-p.y(),2));
   double noisy_range = AddNoise(mNoise.x(), range);
   double x = range*cos(orig_angle-p.theta());
   double y = range*sin(orig_angle-p.theta());
   double bearing= atan(y/x);
  //  fprintf(stderr, "atan(-2/0.1) = %f. atan(2/0.1) = %f\n", atan(-2/0.1), atan(2/0.1));
  //  fprintf(stderr, "Range: %f, Bearing: %f, Original Angle: %f\n", range, bearing, orig_angle);
  //  fprintf(stderr, "x = %f, y = %f\n", x, y);
   double noisy_angle = AddNoise(mNoise.y(), bearing);
   Rot2 noisy_bearing = Rot2::fromAngle(AddNoise(mNoise.y(), bearing));
   if (x > 0){
      //fprintf(stderr, "Actually adding landmark\n");
      graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(Symbol('x',pose_no), Symbol('l',landmark_no), noisy_bearing, noisy_range, measurementNoise);
      //fprintf(stderr, "Pose no: %d, Landmark no: %d\n", pose_no, landmark_no);
      return estimate_from_range(poses[pose_no].position, noisy_range, noisy_angle);
   }
    return Point2(20.0, 20.0);
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

bool keyInKeyVector(const KeyVector& keys, Key k){
  //Returns if a key is in the key vector
  for (auto key: keys){
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
}

void Marginalize(const gtsam::Key &key, gtsam::NonlinearFactorGraph& graph, const gtsam::Values &linearization_point) {

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

Values OnePass(gtsam::NonlinearFactorGraph& graph, int pass_no){
  if (pass_no == 0){
    //fprintf(stderr, "Trying to add first pose\n");
    graph.addPrior(Symbol('x',0), Pose2(0,0,0), priorNoise);  // add directly to graph
    //fprintf(stderr, "line 1\n");
    RobotPose p(Symbol('x', 0), Pose2(0.0,0.0,0.0));
    poses.push_back(p);
    //fprintf(stderr, "Added first pose\n");
  }
  for (int i = 0; i < num_poses-1; i++){
    //fprintf(stderr, "pose number = %d\n", i);
    //Add a factor between the two poses
    Pose2 odometry = Odometry(poses[i].position, ground_truth_poses[i+1]);
    //Add noise to ground truth odometry to simulate real life noise
    Pose2 noisy_odometry = Pose2(AddNoise(oNoise.x(), odometry.x()), AddNoise(oNoise.y(), odometry.y()), AddNoise(oNoise.z(), odometry.theta()));
    //Factor between two consecutive poses
    int s = poses.size();
    Pose2 estimate = estimate_from_odom(poses[s-1].position, noisy_odometry);
    poses.push_back(RobotPose(Symbol('x', s), estimate));
    // fprintf(stderr, "Noisy odometry: (%f, %f)\n", noisy_odometry.x(), noisy_odometry.y());
    // fprintf(stderr, "Previous estimate: (%f, %f)\n", poses[s-1].position.x(), poses[s-1].position.y());
    // fprintf(stderr, "Position at index %d initialized as (%f, %f, %f)\n", s, estimate.x(), estimate.y(), estimate.theta());
    graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x',s-1), Symbol('x',s), noisy_odometry, odometryNoise);

    //Factors between landmarks
    for (int landmark_no = 0; landmark_no < landmarks.size(); landmark_no++){
      //fprintf(stderr, "landmark number = %d\n", landmark_no);
      //Returns whether a false positive has been detected on a ground truth or not
      int p = Persistence(landmark_no);
      Point2 estimated_landmark_pos;
      if(p==1){ 
        estimated_landmark_pos = addBearingRangeFactor(i, landmarks.size()-1, graph); 
        if(estimated_landmark_pos.x() != 20.0){landmarks[landmarks.size()-1].position = estimated_landmark_pos;}
      }
      if(p==0){ 
        estimated_landmark_pos = addBearingRangeFactor(i, landmark_no, graph); 
        if(estimated_landmark_pos.x() != 20.0){landmarks[landmark_no].position = estimated_landmark_pos;}
      }
      if(p==2 or p==3){ continue; }
    }
  }
  //fprintf(stderr, "Adding the following keys into current estimate\n");
  Values currentEstimate;
  for(gtsam::Symbol key:graph.keys()){
    if(key.chr()=='x') { currentEstimate.insert(Symbol('x',key.index()), poses[key.index()].position); }
    if(key.chr()=='l') { currentEstimate.insert(Symbol('l',key.index()), landmarks[key.index()].position);}
    key.print();
  }

  currentEstimate.print("Current estimate:");
  graph.print("Graph:");
  LevenbergMarquardtOptimizer optimizer(graph, currentEstimate);
  Values new_result = optimizer.optimize();
  //Updating current landmark and pose estimates
  new_result.print("Before erasing");
  for (gtsam::Symbol key:graph.keys()){
    if(key.chr()=='x') { poses[key.index()].position = new_result.at<Pose2>(key);}
    if(key.chr()=='l') {
        //Probability that landmark actually appears in map
        double prob = landmarks[key.index()].filter.predict(landmarks[key.index()].t);
        fprintf(stderr, "Probability for landmark %d = %f\n", key.index(), prob);
        if(prob > 0.75){
            landmarks[key.index()].position = new_result.at<Point2>(key);
        }
        else{
            new_result.erase(key);
        }
    }

  }
  new_result.print("After erasing");
  return new_result;

}

int main(int argc, char** argv) {
  // AddGroundTruthLandmark(Point2(3.0,2.0), Book);
  // AddGroundTruthLandmark(Point2(1.0,3.0), Book);
  // AddGroundTruthLandmark(Point2(4.0,1.0), Book);
  // NonlinearFactorGraph graph;
  // OnePass(graph, 0);
  // // num_poses = 1;
  // // ground_truth_poses.clear();
  // // ground_truth_poses.push_back(Vector3(5.5,6.0,0.0));
  // OnePass(graph, 1);
  matplotlibcpp::plot({1,3,2,4});
  matplotlibcpp::show();
}
