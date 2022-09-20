#pragma once

#include <ros/ros.h>
#include <fl/Headers.h>
#include <rosdyn_core/primitives.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs_stamped/Float32Stamped.h>

class TrajEstimator
{
public:
  TrajEstimator(ros::NodeHandle nh);
  
  Eigen::Vector6d getVel(); 
  Eigen::Vector6d getDwrench(); 
  
  void wrenchCallback  (const geometry_msgs::WrenchStampedConstPtr& msg );
  void alphaCallback   (const std_msgs_stamped::Float32StampedConstPtr& msg );
  void dWrenchCallback (const geometry_msgs::WrenchStampedConstPtr& msg );
  void velocityCallback(const geometry_msgs::TwistStampedConstPtr& msg );
  void currPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg );
  double evaluateFis   (double dforce, double vel );
  bool updatePoseEstimate(geometry_msgs::PoseStamped& ret); 

private:
  ros::NodeHandle nh_;
  
  rosdyn::ChainPtr chain_bt_;
  ros::Publisher al_pub_;
  
  Eigen::Vector6d w_b_;
  Eigen::Vector6d velocity_;
  Eigen::Vector6d dW_;
  
  bool init_pos_ok;
  
  geometry_msgs::PoseStamped cur_pos_;
  geometry_msgs::PoseStamped init_pose_;
  geometry_msgs::PoseStamped last_pose_;
  
  double max_fl_;
  double min_fl_;
  double alpha_;
  
  double dt_;
  
  fl::Engine*         engine_;
  fl::InputVariable*  d_force_ ;
  fl::InputVariable*  vel_ ;
  fl::OutputVariable* assistance_ ;
  
  
};

