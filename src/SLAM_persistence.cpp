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



/*To change the positions or ground_truth_landmarks:
    - Change ground_truth_num_landmarks
    - Change num_poses
    - Changed ground_truth_poses
    - Change ground_truth_landmarks
    - Change initial_estimate_psoes
    - Change initial_estimate_landmarks
*/

using namespace std;
using namespace gtsam;

bool debug = false;

const int ground_truth_num_landmarks = 3;
const int max_num_landmarks = 6;
const int num_poses = 7;

static Point3 ground_truth_poses[num_poses] = {Vector3(0.0, 0.0, 0.0), Vector3(1.0, 1.0, 0.0), Vector3(2.0, 2.0, 0.0), Vector3(3.0, 3.0, 0.0), Vector3(4.0, 4.0, 0.0), Vector3(5.0, 5.0, 0.0), Vector3(0.0,0.0,0.0)};

Point3 ground_truth_landmarks[ground_truth_num_landmarks] = {Vector3(2.0,1.0,0.0), Vector3(2.0,3.0,0.0), Vector3(3.0,5.0,0.0)};

Pose2 initial_estimate_poses[num_poses] = {Pose2(3.07, 2.49, 0.73), Pose2(1.58, 4.3, 2.72), Pose2(0.44, 3.78, -2.05319), Pose2(2.09, 4.4, 1.65), Pose2(4.92, 0.42, -1.41319), Pose2(0.03, 3.27, 2.29), Pose2(2.09, 4.4, 1.65)};
  
Point2 initial_estimate_landmarks[ground_truth_num_landmarks] = {Point2(3.4, 1.12), Point2(3.03, 1.69), Point2(2.09, 1.57)};

NonlinearFactorGraph graph;

static Vector3 odometry(1.0, 1.0, 0.0);

static Vector2 mNoise(0.01, 0.02);

static Vector3 oNoise(0.2, 0.2, 0.1);

static double lambda_u = 1;
static double lambda_l = .01;

//P_M: Probability of missed detection (object exists, but is not detected)
static double P_M = 0.01;
//P_F: Probability of false positive (object doesn't exist, but is detected)
static double P_F = 0.01;
//time counter for the persistence filter
double t = 1.0;

int passes = 0;

Values initialEstimate2;

std::uniform_real_distribution<double> distribution(0.0, 5.0);
static std::default_random_engine generator(time(0));

std::function<double(double)> logS_T = std::bind(log_general_purpose_survival_function, std::placeholders::_1, lambda_l, lambda_u);

//Array of persistence filters for the number of landmarks
PersistenceFilter landmark_filters[max_num_landmarks] = {PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T), PersistenceFilter(logS_T)};
//Map with feature locations as keys and symbol indices as values
map<int, Vector3> index_to_feature_map;
//stores current symbol_index
int current_landmark_number = ground_truth_num_landmarks;
int current_pose_index = 0;

double h(Vector3 v){
  /*This method hashes a vector to use it as a key in the map*/
  return v.x()+v.y()+v.z();
}

//This method adds noise to the provided measurement to mimic a real life measurement with noisy equipment
double add_noise(double stddev, double measurement){
    double mean = 0.0;
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
       fprintf(stderr, "Vector: (%f, %f, %f) detected\n", detected_feature.x(), detected_feature.y(), detected_feature.z());
       return element.first;
     }
  }
  if(debug){
    fprintf(stderr, "Adding new feature\n");
  }
  index_to_feature_map[current_landmark_number]= detected_feature;
  current_landmark_number++;
  return current_landmark_number-1;
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
      if(debug){
        fprintf(stderr, "Vector: (%f, %f, %f), in ground truth\n", v1.x(), v1.y(), v1.z());
      }
      return true;
    }
  }
  return false;
}

//This method applies the persistence filter to a point that is in the ground truth
bool persistence(int landmark_no){
  //Landmark no: Indicates landmark 1, landmark 2, landmark 3, etc
  if (ground_truth(index_to_feature(landmark_no))){
    double random = 60;//rand()%100;
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
    if(debug){
      fprintf(stderr, "landmark no = %d. This is a fake landmark\n", landmark_no);
    }
  }
  if (landmark_filters[landmark_no].predict(t) > 0.75){
    return true;
  }
  return false;
}

void one_pass() {
  if(debug){
  fprintf(stderr, "Pass %d: \n", passes);
  }
  Vector3 pNoise(0.0,0.0,0.0);
    auto priorNoise = noiseModel::Diagonal::Sigmas(
      pNoise);
    Pose2 prior(0.0, 0.0, 0.0);    
  auto odometryNoise = noiseModel::Diagonal::Sigmas(
      oNoise);
   
  auto measurementNoise = noiseModel::Diagonal::Sigmas(
      mNoise);
  // Adding prior
  if (passes == 0){
     graph.addPrior(Symbol('x',current_pose_index), prior, priorNoise);  // add directly to graph
  }
  for (int i = 0; i < num_poses-1; i++){
    //Add a factor between the two poses
    Vector3 odometry(ground_truth_poses[i+1].x()-ground_truth_poses[i].x(), ground_truth_poses[i+1].y()-ground_truth_poses[i].y(), ground_truth_poses[i+1].z()-ground_truth_poses[i].z());
    Pose2 noisy_odometry = Pose2(add_noise(oNoise.x(), odometry.x()), add_noise(oNoise.y(), odometry.y()), add_noise(oNoise.z(), odometry.z()));
    graph.emplace_shared<BetweenFactor<Pose2> >(Symbol('x',current_pose_index), Symbol('x',current_pose_index+1), noisy_odometry, odometryNoise);
    int map_size = index_to_feature_map.size();
    for (int j = 0; j <map_size; j++){
      bool p = persistence(j);
      if(debug){
         fprintf(stderr, "p = %d, i = %d, j = %d\n", p, i, j);
      }
      Vector3 feat = index_to_feature(j);
      if (feat.x() >= ground_truth_poses[i].x() && p){
        double angle = atan((feat.y()-ground_truth_poses[i].y())/(feat.x()-ground_truth_poses[i].x()));
        Rot2 bearing = Rot2::fromAngle(add_noise(mNoise.y(), angle));
        //bearing.print();
        double range = sqrt(pow(feat.y()-ground_truth_poses[i].y(), 2.0)+pow(feat.x()-ground_truth_poses[i].x(), 2.0));
        double noisy_range = add_noise(mNoise.x(), range);
        //fprintf(stderr, "Adding factor: i: %d,j: %d\n", i, j);
        graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(Symbol('x',current_pose_index), Symbol('l',j), bearing, noisy_range, measurementNoise);
      }
    }
    current_pose_index++;
    t+=1.0;
  }
  Values initialEstimate;
  fprintf(stderr, "Before initial estimate\n");
  if(passes ==0){
    initialEstimate2.insert(Symbol('x',0), initial_estimate_poses[0]);
  for (int i = 0; i < ground_truth_num_landmarks; i++){
    initialEstimate2.insert(Symbol('l',i), initial_estimate_landmarks[i]);
   }
  }
  for (int i = 1; i < num_poses; i++){
      initialEstimate2.insert(Symbol('x',i+passes*(num_poses-1)), initial_estimate_poses[i]);
  }

  //fprintf(stderr, "initial estimate 2\n");
  //initialEstimate2.print("Initial estimate:");
  //graph.print();
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate2);
  Values result = optimizer.optimize();
  //result.print("Final: ");

  //Replace initial_estimate with current SLAM values
  //fprintf(stderr, "optimizer done\n");
  for (int i = 0; i < num_poses; i++){
     initial_estimate_poses[i] = result.at<Pose2>(Symbol('x', i+passes*(num_poses-1)));
  }
  for (int i = 0; i < current_landmark_number; i++){
     initial_estimate_landmarks[i] = result.at<Point2>(Symbol('l', i));
  }
  //fprintf(stderr, "end\n");
  //Increase the number of passes through the board
  passes++;
}


int main(int argc, char** argv) {
    // Create the keys we need
  
  for (int i = 0; i < ground_truth_num_landmarks; i ++){
    index_to_feature_map[i] = ground_truth_landmarks[i];
  }

   one_pass();
   ground_truth_landmarks[0] = Vector3(3.1, 2.0, 0.0);
   index_to_feature_map[3] = index_to_feature_map[2];
   index_to_feature_map[2] = ground_truth_landmarks[0];
   current_landmark_number++;
   debug = true;
   one_pass();
   one_pass();
   one_pass();
   one_pass();
   one_pass();

}
