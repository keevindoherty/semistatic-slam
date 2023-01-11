#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <persistence_filter/c++/include/persistence_filter.h>
#include <persistence_filter/c++/include/persistence_filter_utils.h>

using namespace gtsam;
using namespace std;

//File good?

enum SemanticType{
  Book, 
  Chair,
  Table,
  Bed
};
//Table survival time: 200
//Chair survival time: 50
//Book survival time: 30
//Bed survival time: 

class RobotPose{
  public:
    RobotPose(const Symbol& s, const Pose2& pos) {
       symbol = s;
       position = pos;
    }
    void print(){
      fprintf(stderr, "Robot Pose. Symbol: Symbol('l',%d), position: (%f, %f)\n", symbol.index(), position.x(), position.y());
    }
    Pose2 getPosition(){
      return position;
    }
    void changePosition(const Pose2& p){
      position = p;
    }
    Symbol getSymbol(){
      return symbol;
    }

  private:
    gtsam::Pose2 position;
    gtsam::Symbol symbol;
};

class Landmark{
  public:
    //category indicates if position is a landmark or pose
    //PersistenceFilter filter{logS_T};
    double lambda_l;
    double lambda_u;
    PersistenceFilter filter{std::bind(log_general_purpose_survival_function, std::placeholders::_1, 0, 1)};
    double t;

    Landmark(const gtsam::Symbol& s, const gtsam::Point2& pos, const SemanticType& sem) {
      symbol = s;
      current_position = pos;
      semanticType = sem;
      t = 1.0;
      switch(sem){
        case Table:
          lambda_l = 0.001;
          lambda_u = 0.005;
          break;
        case Chair:
          lambda_l = 0.005;
          lambda_u = 0.02;
          break;
        case Book:
          lambda_l = 0.01;
          lambda_u = 0.04;
          break;
      }
      filter = PersistenceFilter{std::bind(log_general_purpose_survival_function, std::placeholders::_1, Landmark::lambda_l, Landmark::lambda_u)};
    };
    void incrementTime(){
      t++;
    }
    void print(){
      fprintf(stderr, "Landmark. Symbol: Symbol('l',%d), position: (%f, %f), time: %f\n", symbol.index(), current_position.x(), current_position.y(), t);
    }
    Point2 getPosition(){
      return current_position;
    }
    void setPosition(const Point2& new_pos){
      current_position = new_pos;
    }
    void inGroundTruth(){
      groundTruth = true;
    }
    Symbol getSymbol(){
      return symbol;
    }
    SemanticType getSemanticType(){
      return semanticType;
    }
    void setSemanticType(const SemanticType& s){
      semanticType = s;
    }

    virtual double getPersistence()=0;

  private:
    gtsam::Point2 current_position;
    bool groundTruth = false;
    gtsam::Symbol symbol;
    SemanticType semanticType;
};

class LandmarkPersistence : public Landmark{
  public:
    LandmarkPersistence(const gtsam::Symbol& s, const gtsam::Point2& pos, const SemanticType& sem):Landmark(s, pos, sem){}; 
    double getPersistence(){
      return filter.predict(t);
    }
};

class LandmarkNoPersistence : public Landmark{
    public:
    LandmarkNoPersistence(const gtsam::Symbol& s, const gtsam::Point2& pos, const SemanticType& sem):Landmark(s, pos, sem){}; 
    double getPersistence(){
      return 0;
    }
};


class Observation{
  public:
    Observation(const gtsam::Point2& pos, const SemanticType& sem) {
      position = pos;
      semanticType = sem;
    }

    void AssociateLandmark(const gtsam:: Symbol& s){
      associatedLandmark = s;
    }

    Symbol getAssociatedLandmark(){
      return associatedLandmark;
    }

    Point2 getPosition(){
      return position;
    }

    void setPosition(const Point2& p){
      position = p;
    }

    SemanticType getSemanticType(){
      return semanticType;
    }

    void setSemanticType(const SemanticType& s){
      semanticType = s;
    }

    void print(){
      fprintf(stderr, "Observation. Position: (%f, %f). Symbol: ", position.x(), position.y());
      associatedLandmark.print();
    }

  private:
    Symbol associatedLandmark;
    Point2 position;
    SemanticType semanticType;
};

struct plots{
  vector<vector<double>> x_landmark_plot;
  vector<vector<double>> y_landmark_plot;
  vector <double> x_pose_plot;
  vector <double> y_pose_plot;
};

int num_poses = 21;
int num_ground_truth_landmarks = 0;
//Ground truth landmarks
vector<LandmarkPersistence> ground_truth_landmarks_persistence;
vector<LandmarkNoPersistence> ground_truth_landmarks_no_persistence;
//Given in x-y-z coordinates
vector <Vector3> ground_truth_poses = {Vector3(0.0,0.0,0.0), Vector3(1.0,1.0,0.0), Vector3(2.0,2.0,0.0), Vector3(3.0,3.0,0.0), Vector3(4.0,4.0,0.0), Vector3(5.0,5.0,0.0), Vector3(6.0,6.0,0.0), Vector3(7.0,7.0,0.0), Vector3(8.0,8.0,0.0), Vector3(9.0,9.0,0.0), Vector3(10.0,10.0,0.0), Vector3(11.0,11.0,0.0), Vector3(12.0,12.0,0.0), Vector3(13.0,13.0,0.0), Vector3(14.0,14.0,0.0), Vector3(15.0,15.0,0.0), Vector3(16.0,16.0,0.0), Vector3(17.0,17.0,0.0), Vector3(18.0,18.0,0.0), Vector3(19.0,19.0,0.0), Vector3(20.0,20.0,0.0)};
vector<LandmarkPersistence> landmarks_persistence;
vector<LandmarkNoPersistence> landmarks_no_persistence;
vector<RobotPose> poses;
vector<Observation> observations;



