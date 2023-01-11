#pragma once
#include "matplotlibcpp.h"

using namespace std;

int red_overlay = 2;

void MakePlot(plots& pl, const int& pass_no, vector<Landmark>& landmarks){
  //int reduction = (pass_no+1) * 2;
  if(pass_no == 0){
    red_overlay = pl.x_landmark_plot.size()-2;
  }
  for(int pose_no = 0; pose_no < pl.x_landmark_plot.size(); pose_no++){
    for (int landmark_no = 0; landmark_no< pl.x_landmark_plot[pose_no].size(); landmark_no++){
      vector<double> v5 = {pl.x_pose_plot[pose_no],pl.x_landmark_plot[pose_no][landmark_no]};
      vector<double> v6 = {pl.y_pose_plot[pose_no],pl.y_landmark_plot[pose_no][landmark_no]};
      matplotlibcpp::plot(v5, v6, "go-", {{"linewidth", "0.5"}});
    }
  }
  int pose_no = red_overlay;
  for (int landmark_no = 0; landmark_no< pl.x_landmark_plot[pose_no].size(); landmark_no++){
      vector<double> v5 = {pl.x_pose_plot[pose_no],pl.x_landmark_plot[pose_no][landmark_no]};
      vector<double> v6 = {pl.y_pose_plot[pose_no],pl.y_landmark_plot[pose_no][landmark_no]};
      matplotlibcpp::plot(v5, v6, "ro-", {{"linewidth", "0.5"}});
    }
  for (auto landmark: landmarks){
      vector<double> v5 = {landmark.getPosition().x()};
      vector<double> v6 = {landmark.getPosition().y()};
      matplotlibcpp::plot(v5, v6, "mo");
  }
  matplotlibcpp::plot(pl.x_pose_plot, pl.y_pose_plot,"bo");
  matplotlibcpp::xlim(0, 20);
  matplotlibcpp::ylim(0, 20);
  matplotlibcpp::grid();
  matplotlibcpp::show();
}

void PlotIntermediateResult(Values result){
  vector<double> v5;
  vector<double> v6;
  vector<double> v7;
  vector<double> v8;
   for (auto pair:result){
    gtsam::Symbol k = pair.key;
    if(k.chr()=='x') { 
      v5.push_back(result.at<Pose2>(k).x());
      v6.push_back(result.at<Pose2>(k).y());
    }
    if(k.chr()=='l') {
      v5.push_back(result.at<Pose2>(k).x());
      v6.push_back(result.at<Pose2>(k).y());
    }
  }
}