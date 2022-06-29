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
#include <matplotlibcpp.h>
#include <math.h>

using namespace std;
using namespace gtsam;

class Position{
    public:
    gtsam::Symbol symbol;
    gtsam::Vector3 position;
    //category indicates if position is a landmark or pose
    string category;
    PersistenceFilter filter = PersistenceFilter(logS_T);
    double t;

    Position() {}
    Position(const Symbol& s, const Vector3& pos, string cat){
       symbol = s;
       position = pos;
       category = cat;
    }

    private:
    const double lambda_u = 1;
    const double lambda_l = .01;
    std::function<double(double)> logS_T = std::bind(log_general_purpose_survival_function, std::placeholders::_1, lambda_l, lambda_u);
};

//GLOBAL VARIABLES
double random_seed = 12.0;
std::uniform_real_distribution<double> distribution(0.0, 5.0);
static std::default_random_engine generator(random_seed);
//Vectors of class Position for landmarks and poses
vector<Position> landmarks;
vector<Position> poses;
//Number of poses (per pass) and number of ground truth landmarks
const int num_poses = 7;
int num_ground_truth_landmarks = 0;
//Ground truth landmarks
vector<Position> ground_truth_landmarks;
Vector3 ground_truth_poses[num_poses] = {Vector3(0.0,0.0,0.0), Vector3(1.0,1.0,0.0), Vector3(2.0,2.0,0.0),
                                        Vector3(3.0,3.0,0.0), Vector3(4.0,4.0,0.0), Vector3(5.0,5.0,0.0),
                                        Vector3(0.0,0.0,0.0)};
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
static double P_M = 0.1;
//P_F: Probability of false positive (object doesn't exist, but is detected)
static double P_F = 0.1;
//time counter for the persistence filter
vector <double> t;
//Vector of recently updated ground truth symbols (ie landmarks with this symbol are currently set to ground truth position)
//This vector is cleared every iteration, and landmarks from this iteration are updated

double AddNoise(double stddev, double measurement){
    double mean = 0.0;
    std::normal_distribution<double> dist(mean, stddev);

    // Add Gaussian noise
    return measurement+dist(generator);
}

void AddGroundTruthLandmark(const Vector3& pos){
    Position p(Symbol('l',landmarks.size()), pos, "landmark");
    landmarks.push_back(p);
    ground_truth_landmarks.push_back(p);
}

void MoveGroundTruthLandmark(Vector3 new_pos, int landmark_no){
    fprintf(stderr, "MoveGroundTruthLandmark not yet defined\n");
}

double Distance(Vector3 v1, Vector3 v2){
  return sqrt(pow(v1.x()-v2.x(),2.0)+pow(v1.y()-v2.y(),2.0));
}

bool GroundTruth(Vector3 v){
    //Returns if vector is in ground truth
    for (Position l: ground_truth_landmarks){
        if(Distance(l.position, v)<0.1){
            return true;
        }
    }
    return false;
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
    }
    else{
      //If a false positive/negative aren't being simulated, then the landmark is marked as true
      landmarks[landmark_no].filter.update(true, landmarks[landmark_no].t, P_M, P_F);
    }
    if (false_positive){
      //Add a false positive landmark to landmark list
      landmarks.push_back(Position(Symbol('l',landmarks.size()), Vector3(distribution(generator),distribution(generator),distribution(generator)), "landmark"));
      //Add corresponding time stamp
      t.push_back(0.0);
      landmarks[landmarks.size()-1].filter.update(true, landmarks[landmarks.size()-1].t, P_M, P_F);
      landmarks[landmarks.size()-1].t++;
    }
    landmarks[landmark_no].t++;
    if (false_positive){ return 1; }
    if(false_negative){ return 3; }
  //Landmark in the ground truth, not false positive or negative
  return 0;
  }
  else{
    //Running persistence on a previous false positive
    landmarks[landmark_no].filter.update(false, t[landmark_no], P_M, P_F);
    t[landmark_no]++;
    fprintf(stderr, "Running persistence on a false positive. Updating landmark %d with false\n", landmark_no);
    return 2;
  }
}

Vector3 estimate_from_odom(Vector3 past_pos, Pose2 odometry){
   return Vector3(past_pos.x()+odometry.x(), past_pos.y()+odometry.y(), past_pos.z());
}

Vector3 estimate_from_range(Vector3 past_pos, double range, double bearing){
  Vector3 diff(1/sqrt(1 + pow(tan(bearing),2))*range,tan(bearing)/sqrt(1 + pow(tan(bearing),2))*range,0);
  return Vector3(past_pos.x()+diff.x(), past_pos.y()+diff.y(), past_pos.z()+diff.z());
}

Vector3 addBearingRangeFactor(int pose_no, int landmark_no, NonlinearFactorGraph& graph){
  //Returns approximate location of landmark from measurement (noisy range and bearing)
   fprintf(stderr, "Position: %d, Landmark: %d\n", pose_no, landmark_no);
   Vector3 feat = landmarks[landmark_no].position;
   if (feat.x() >= poses[pose_no].position.x()){
        double angle = atan((feat.y()-poses[pose_no].position.y())/(feat.x()-poses[pose_no].position.x()));
        double noisy_angle = AddNoise(mNoise.y(), angle);
        Rot2 noisy_bearing = Rot2::fromAngle(AddNoise(mNoise.y(), angle));
        double range = sqrt(pow(feat.y()-poses[pose_no].position.y(), 2.0)+pow(feat.x()-poses[pose_no].position.x(), 2.0));
        double noisy_range = AddNoise(mNoise.x(), range);
        fprintf(stderr, "Actually adding landmark\n");
        graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(Symbol('x',pose_no), Symbol('l',landmark_no), noisy_bearing, noisy_range, measurementNoise);
        return estimate_from_range(poses[pose_no].position, noisy_range, noisy_angle);
   }
   else{
    return Vector3(20.0,20.0,20.0);
   }
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
    fprintf(stderr, "Trying to add first pose\n");
    graph.addPrior(Symbol('x',0), Vector3(0,0,0), priorNoise);  // add directly to graph
    fprintf(stderr, "line 1\n");
    // Position p(Symbol('x', 0), Vector3(0.0,0.0,0.0), "pose");
    Symbol s('x', 0);
    fprintf(stderr, "line 1-1\n");
    Vector3 v3(0.0,0.0,0.0);
    fprintf(stderr, "line 1-2\n");
    Position p;
    fprintf(stderr, "line2\n");
    poses.push_back(Position(Symbol('x', 0), Vector3(0.0,0.0,0.0), "pose"));
    fprintf(stderr, "Added first pose\n");
  }
  for (int i = 0; i < num_poses-1; i++){
    fprintf(stderr, "pose number = %d\n", i);
    //Add a factor between the two poses
    Vector3 odometry(ground_truth_poses[i+1].x()-ground_truth_poses[i].x(), ground_truth_poses[i+1].y()-ground_truth_poses[i].y(), ground_truth_poses[i+1].z()-ground_truth_poses[i].z());
    //Add noise to ground truth odometry to simulate real life noise
    Pose2 noisy_odometry = Pose2(AddNoise(oNoise.x(), odometry.x()), AddNoise(oNoise.y(), odometry.y()), AddNoise(oNoise.z(), odometry.z()));
    //Factor between two consecutive poses
    int s = poses.size();
    poses.push_back(Position(Symbol('x', s+1), estimate_from_odom(poses[s].position, noisy_odometry), "pose"));
    graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x',s), Symbol('x',s+1), noisy_odometry, odometryNoise);

    //Factors between landmarks
    for (int landmark_no = 0; landmark_no < landmarks.size(); landmark_no++){
      fprintf(stderr, "landmark number = %d\n", landmark_no);
      //Returns whether a false positive has been detected on a ground truth or not
      int p = Persistence(landmark_no);
      Vector3 estimated_landmark_pos;
      if(p==1){ estimated_landmark_pos = addBearingRangeFactor(i, landmarks.size()-1, graph); }
      if(p==0){ estimated_landmark_pos = addBearingRangeFactor(i, landmark_no, graph); }
      if(p==2 or p==3){ continue; }
      if(estimated_landmark_pos.x()!=20.0){
        landmarks[landmark_no].position = estimated_landmark_pos;
      }
    }
  }
  fprintf(stderr, "Adding the following keys into current estimate\n");
  Values currentEstimate;
  for(gtsam::Symbol key:graph.keys()){
    if(key.chr()=='x') { currentEstimate.insert(Symbol('x',key.index()), poses[key.index()].position);}
    if(key.chr()=='l') { currentEstimate.insert(Symbol('l',key.index()), landmarks[key.index()].position);}
    key.print();
  }
  currentEstimate.print("Current estimate:");
  LevenbergMarquardtOptimizer optimizer(graph, currentEstimate);
  Values new_result = optimizer.optimize();
  //Updating current landmark and pose estimates
  for (gtsam::Symbol key:graph.keys()){
    if(key.chr()=='x') { poses[key.index()].position = Vector3(new_result.at<Pose2>(key).x(), new_result.at<Pose2>(key).y(), 0.0);}
    if(key.chr()=='l') {
        //Probability that landmark actually appears in map
        double prob = landmarks[key.index()].filter.predict(landmarks[key.index()].t);
        if(prob > 0.75){
            landmarks[key.index()].position = Vector3(new_result.at<Pose2>(key).x(), new_result.at<Pose2>(key).y(), 0.0);
        }
        else{
            new_result.erase(key);
        }
    }
  }
  new_result.print();
  return new_result;

}

int main(int argc, char** argv) {
  AddGroundTruthLandmark(Vector3(3.0,2.0,0.0));
  AddGroundTruthLandmark(Vector3(1.0,3.0,0.0));
  AddGroundTruthLandmark(Vector3(4.0,1.0,0.0));
  NonlinearFactorGraph graph;
  #if 0
  Vector3 pos(4.0,1.0,0.0);
  Position p(Symbol('l',landmarks.size()), pos, "landmark");
  #endif
  OnePass(graph, 0);
}