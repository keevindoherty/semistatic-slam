#pragma once
#include "LandmarkPoses.h"
#include "Odometry.h"
#include <hungarian-algorithm-cpp/Hungarian.h>
#include <hungarian-algorithm-cpp/Hungarian.cpp>

using namespace gtsam;
using namespace std;

double costFactorDist = 3.0;
double costFactorAngle = 0.4;
double costFactorType = 1.0;
double costFactorPersistence = 5.0;
//Delete from landmarks database if too close to each other and same semantic type

double Cost(Pose2 pos, Landmark existing, Observation observed){
  //Changing data types to Pose2 and Vector3 to pass into Odometry() function
  Pose2 transform1 = RelativeOdometry(pos, Vector3(existing.getPosition().x(), existing.getPosition().y(), 0.0));
  Pose2 transform2 = RelativeOdometry(pos, Vector3(observed.getPosition().x(), observed.getPosition().y(), 0.0));
  double dist = sqrt(pow(transform1.x()-transform2.x(),2)+pow(transform1.y()-transform2.y(),2));
  double angle = abs(transform1.theta()-transform2.theta());
  bool sametype = false;
  if(observed.getSemanticType() == existing.getSemanticType()){ sametype = true; }
  return floor((costFactorDist*dist + costFactorAngle*angle + costFactorType*(1-sametype)+costFactorPersistence*existing.getPersistence())*100);
}

vector<vector<double>> ConstructCostMatrix(Pose2 pos, vector<Landmark>& landmarks, vector<Observation>& new_observations){
  vector <vector<double>> costMatrix;
  for (Landmark landmark: landmarks){
    vector<double> v;
    costMatrix.push_back(v);
    for (int i = 0; i < new_observations.size(); i++){
      costMatrix[costMatrix.size()-1].push_back(Cost(pos, landmark, new_observations[i]));
    }
  }
  // PRINT COST MATRIX HERE:
  // fprintf(stderr, "Cost Matrix\n");
  // for (int i = 0; i < costMatrix.size(); i++){
  //   for(int j = 0; j < costMatrix[i].size(); j++){
  //     fprintf(stderr, "%f, ", costMatrix[i][j]);
  //   }
  //}

  return costMatrix;
}

void LandmarkAssociation(Pose2 pos, vector<Landmark>& landmarks, vector<Observation>& new_observations){
  vector<vector<double>> costMatrix = ConstructCostMatrix(pos, landmarks, new_observations);
  HungarianAlgorithm HungAlgo;
  vector<int> assignment;
  HungAlgo.Solve(costMatrix, assignment);
  for (int a=0; a<assignment.size(); a++){
    if(assignment[a] >= 0){  
      new_observations[assignment[a]].AssociateLandmark(Symbol('l',a));
      //fprintf(stderr, "landmark number = %d, observation number: %d\n", a, assignment[a]);
    }
  }
  //fprintf(stderr, "End\n");
}
