#pragma once

#include <ros/ros.h>
#include <fl/Headers.h>
#include <rosdyn_core/primitives.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>

class TrajEstimator
{
public:
  TrajEstimator(ros::NodeHandle nh);
  
  Eigen::Vector6d getDerr(); 
  Eigen::Vector6d getDwrench(); 
  
  void wrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg );
  void velocityCallback(const geometry_msgs::TwistStampedConstPtr& msg );
  double evaluateFis(const double dforce, double derr );
  
private:
  ros::NodeHandle nh_;
  
  rosdyn::ChainPtr chain_bt_;
  ros::Publisher al_pub_;
  
  Eigen::Vector6d w_b_;
  Eigen::Vector6d err_b_;
  
  Eigen::Vector6d d_w_b_;
  Eigen::Vector6d d_err_b_;
  
  double max_fl_;
  double min_fl_;
  
  double dt_;
  
  fl::Engine*         engine_;
  fl::InputVariable*  d_force_ ;
  fl::InputVariable*  d_err_ ;
  fl::OutputVariable* assistance_ ;
  
  
};

