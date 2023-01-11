#pragma once
#include <math.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "Random.h"

using namespace std;
using namespace gtsam;

static Vector3 pNoise(0.0,0.0,0.0);
static Vector2 mNoise(0.05, 0.05);
static Vector3 oNoise(0.06, 0.06, 0.01);

const auto priorNoise = noiseModel::Diagonal::Sigmas(
    pNoise);
const auto odometryNoise = noiseModel::Diagonal::Sigmas(
    oNoise);   
const auto measurementNoise = noiseModel::Diagonal::Sigmas(
    mNoise);

double AddNoise(double stddev, double measurement){
    double mean = 0.0;
    normal_distribution<double> dist(mean, stddev);
    // Add Gaussian noise
    return measurement+dist(generator);
}

Pose2 RelativeOdometry(Pose2 p1, Point3 v2){
//Gives the robots odometry given its starting position and ending coordinates
double orig_angle = atan2((v2.y()-p1.y()),(v2.x()-p1.x()));
double r = sqrt(pow(v2.x()-p1.x(),2)+pow(v2.y()-p1.y(),2));
double x = r*cos(orig_angle-p1.theta());
double y = r*sin(orig_angle-p1.theta());
double theta = atan2(y,x);
Pose2 odometry(x, y, theta);
return odometry;
}

Pose2 PoseEstimate(Pose2 past_pos, Pose2 odometry){
  // Return current position estimate given a past position and the relative odometry from the past position (Pose 2)
  double r = sqrt(pow(odometry.x(),2)+pow(odometry.y(),2));
  double x = past_pos.x()+r*cos(past_pos.theta()+odometry.theta());
  double y = past_pos.y()+r*sin(past_pos.theta()+odometry.theta());
  double theta = past_pos.theta()+odometry.theta();
  return Pose2(x,y,theta);
}

Point2 LandmarkEstimate(Pose2 past_pos, double range, double bearing){
  // Return landmark position estimate given a position and the relative odometry from the position (range and bearing)
  double x = past_pos.x()+range*cos(past_pos.theta()+bearing);
  double y = past_pos.y()+range*sin(past_pos.theta()+bearing);
  double theta = past_pos.theta()+bearing;
  return Point2(x,y);
}

double Distance(Point3 p1, Pose2 p2){
  return sqrt(pow(p1.x()-p2.x(),2)+pow(p1.y()-p2.y(),2));
}



