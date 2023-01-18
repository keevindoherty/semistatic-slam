#pragma once
#include "LandmarkPoses.h"
#include "Random.h"
#include "Odometry.h"

using namespace gtsam;
using namespace std;

//File good?

//P_M: Probability of missed detection (object exists, but is not detected)
static double P_M = 0.10;
//P_F: Probability of false positive (object doesn't exist, but is detected)
static double P_F = 0.02;
double pi = atan(1)*4;
double threshold = 0.5;

double Distance(Point2 v1, Point2 v2){
  return sqrt(pow(v1.x()-v2.x(),2.0)+pow(v1.y()-v2.y(),2.0));
}

bool InLandmarks(Point2 v, vector<Landmark*> landmarks){
  //Returns whether an observed point can reasonably be mapped to an existing landmark or if a new Landmark should be made for it
  for (auto l: landmarks){
        if(Distance(l->getPosition(), v)<0.5){
            return true;
        }
    }
    return false;
}

vector<Observation> GenerateObservations(plots& plots, int pose_no, vector<RobotPose>& poses, vector<Landmark*>& landmarks, vector<Observation> &observations, vector<Landmark*>& ground_truth_landmarks, bool persistence){
  vector<Observation> new_observations;
  for(auto& landmark: ground_truth_landmarks){
    double random = distribution(generator)*20.0;
    bool false_positive = false;
    bool false_negative = false;
    if (random < 100*P_F){
      false_positive = true; 
    }
    if (random > 100*(1-P_M)){
      // fprintf(stderr, "False negative, ");
      // landmark.print();
      false_negative = true;
    }
    Pose2 relOdom;
    if (false_negative){
      //False negative
      continue;
    }
    if (false_positive){
      Point2 pos = Point2(distribution(generator),distribution(generator));
      landmark->print();
      fprintf(stderr, "False positive. location: (%f, %f)\n", pos.x(), pos.y());
      if(!InLandmarks(pos, landmarks)){
        if(persistence){
          LandmarkPersistence* l = new LandmarkPersistence(Symbol('l',landmarks.size()-1), pos, Book);
          landmarks.push_back(l);
        }
        else{
          LandmarkNoPersistence* l = new LandmarkNoPersistence(Symbol('l',landmarks.size()-1), pos, Book);
          landmarks.push_back(l);
        }
      }
      relOdom = RelativeOdometry(poses.at(pose_no).getPosition(), Point3(pos.x(), pos.y(), 0.0));
    }
    if (!false_negative && !false_positive){
    relOdom = RelativeOdometry(poses.at(pose_no).getPosition(), Point3(landmark->getPosition().x(), landmark->getPosition().y(), 0.0));
    }
  double range = sqrt(pow(relOdom.x(), 2)+pow(relOdom.y(), 2));
  double noisy_range = AddNoise(mNoise.x(), range);
  double noisy_bearing = AddNoise(mNoise.y(), relOdom.theta());
  Observation o(LandmarkEstimate(poses.at(pose_no).getPosition(), noisy_range, noisy_bearing), Book);
  if(!false_positive){
    //o.AssociateLandmark(landmark->getSymbol());
    o.AssociateLandmark(Symbol('l',landmarks.size()-1));
    //o.print();
  }
  else{
    o.AssociateLandmark(Symbol('l',landmarks.size()-1));
    //o.print();
  }
  if(-pi/2<relOdom.theta() && relOdom.theta()<pi/2){
    new_observations.push_back(o);
    //o.print();
   // fprintf(stderr, "Rel Odom Theta: %f\n", relOdom.theta());
  }
  }

  return new_observations;
}

void UpdateLandmarkPredictions(vector<Landmark*>& landmarks, vector<Observation>& new_observations){
  for (auto& observation: new_observations){
    landmarks.at(observation.getAssociatedLandmark().index())->setPosition(observation.getPosition());
    //fprintf(stderr, "Update landmark %d\n", observation.getAssociatedLandmark().index());
  }
}

void AddPoseLandmarkFactor(int pose_no, vector<RobotPose>& poses, int landmark_no, vector<Landmark*>& landmarks, NonlinearFactorGraph& graph, plots& plots){
  Pose2 relOdom = RelativeOdometry(poses.at(pose_no).getPosition(), Vector3(landmarks.at(landmark_no)->getPosition().x(), landmarks.at(landmark_no)->getPosition().y(), 0.0));
  double range = sqrt(pow(relOdom.x(), 2)+pow(relOdom.y(), 2));
  double bearing = relOdom.theta();
  if (bearing < pi/2 && bearing > -pi/2){
    //fprintf(stderr, "Pose angle: %f, Bearing: %f\n", poses.at(pose_no).getPosition().theta(), bearing);
    plots.x_landmark_plot[pose_no].push_back(landmarks.at(landmark_no)->getPosition().x());
    plots.y_landmark_plot[pose_no].push_back(landmarks.at(landmark_no)->getPosition().y());
    graph.emplace_shared<BearingRangeFactor<Pose2, Point2> >(Symbol('x',pose_no), Symbol('l',landmark_no), bearing, range, measurementNoise);
   }
}

void UpdateGraphFactors(int pose_no, vector<RobotPose>& poses, vector<Landmark*>& landmarks, NonlinearFactorGraph& graph, vector<Observation>& new_observations, plots& plots){
  //Adds factors between position and each observation
  for (auto observation: new_observations){
    AddPoseLandmarkFactor(pose_no, poses, observation.getAssociatedLandmark().index(), landmarks, graph, plots);
  }
  //fprintf(stderr, "Updating graph factors\n");
}

void UpdatePersistence(int pose_no, vector<Landmark*>& landmarks, vector<RobotPose>& poses, vector<Observation>& new_observations){
  vector<int> update;
  //Records all landmarks for which persistence must be updated
  for (auto& observation: new_observations){
    update.push_back(observation.getAssociatedLandmark().index());
  }
  for(auto landmark: landmarks){
    double angle = RelativeOdometry(poses[pose_no].getPosition(), Vector3(landmark->getPosition().x(), landmark->getPosition().y(), 0.0)).theta();
    if(angle<-pi/2 || angle > pi/2){
      landmark->incrementTime();
      // landmark.print();
      // fprintf(stderr, "Landmark not updated with false\n");
      continue;
    }
    //if a given index is not in update, update with false. Else, update with true
    if (find(update.begin(), update.end(), landmark->getSymbol().index()) == update.end()){
      landmark->filter.update(false, landmark->t, P_M, P_F);
    }
    else{
      landmark->filter.update(true, landmark->t, P_M, P_F);
    }
    landmark->incrementTime();
  }
}

void UpdateObservations(vector<Observation>& observations, vector<Observation>& new_observations){
  for(auto& observation: new_observations){
    observations.push_back(observation);
  }
}