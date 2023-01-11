#pragma once
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <iostream>
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
#include <cstdlib>

#include "../SLAM_Headers/LandmarkPoses.h"
#include "../SLAM_Headers/LandmarkUpdates.h"
#include "../SLAM_Headers/Marginalize.h"
#include "../SLAM_Headers/DataAssociation.h"
#include "../SLAM_Headers/Odometry.h"
#include "../SLAM_Headers/Plotting.h"
#include "../SLAM_Headers/Random.h"

