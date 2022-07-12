#include <trajectory_plotter/human_traj_estimation.h>
#include <trajectory_plotter/utils.h>
#include <ros/package.h>

TrajEstimator::TrajEstimator(ros::NodeHandle nh)
:nh_(nh)
{
  w_b_    .setZero();
  err_b_  .setZero();
  d_w_b_  .setZero();
  d_err_b_.setZero();
  
  if ( !nh_.getParam ( "sampling_time", dt_) )
  {
    ROS_WARN_STREAM (nh_.getNamespace() << " /sampling_time set. default 0.008");
    dt_=0.008;
  }
  
  std::string path = ros::package::getPath("trajectory_plotter");
  path += "/config/assistance.fis";
  ROS_INFO_STREAM("recovering path: " << path);
  
  engine_ = fl::FisImporter().fromFile(path);
  std::string status;
  if (not engine_->isReady(&status))
    ROS_ERROR_STREAM("engine is not ready");

  d_force_   = engine_->getInputVariable("dforce");
  d_err_     = engine_->getInputVariable("derr");
  assistance_= engine_->getOutputVariable("assistance");
  
  if ( !nh_.getParam ( "max_fl", max_fl_) )
  {
    max_fl_ = 0.007;
    ROS_WARN_STREAM (nh_.getNamespace() << " /max_fl set. default: " << max_fl_);
  }
}

Eigen::Vector6d TrajEstimator::getDerr() {return d_err_b_;}
Eigen::Vector6d TrajEstimator::getDwrench() {return d_w_b_;}

void TrajEstimator::wrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg )
{
  Eigen::Vector6d old_wb = w_b_;
  
  w_b_( 0 ) = msg->wrench.force.x;
  w_b_( 1 ) = msg->wrench.force.y;
  w_b_( 2 ) = msg->wrench.force.z;
  w_b_( 3 ) = msg->wrench.torque.x;
  w_b_( 4 ) = msg->wrench.torque.y;
  w_b_( 5 ) = msg->wrench.torque.z;
  
  for (int i =0; i < 6; i++)
    d_w_b_(i) = ( w_b_(i)-old_wb(i) )/dt_;
  
  ROS_INFO_STREAM_THROTTLE(1.0,"wrench: "<< w_b_.transpose());
  ROS_INFO_STREAM_THROTTLE(1.0,"d wrench: "<< d_w_b_.transpose());
}


void TrajEstimator::velocityCallback(const geometry_msgs::TwistStampedConstPtr& msg )
{
  Eigen::Vector6d old_err = err_b_;
  
  err_b_( 0 ) = msg->twist.linear.x;
  err_b_( 1 ) = msg->twist.linear.y;
  err_b_( 2 ) = msg->twist.linear.z;
  err_b_( 3 ) = msg->twist.angular.x;
  err_b_( 4 ) = msg->twist.angular.y;
  err_b_( 5 ) = msg->twist.angular.z;
  
  
  for (int i =0; i < 6; i++)
    d_err_b_(i) = ( err_b_(i)-old_err(i) )/dt_;
  
  ROS_INFO_STREAM_THROTTLE(1.0,"error: "<< err_b_.transpose());
  ROS_INFO_STREAM_THROTTLE(1.0,"d error: "<< d_err_b_.transpose());
}


double TrajEstimator::evaluateFis(const double dforce, const double derr )
{
  d_force_->setValue(dforce);
  d_err_->setValue(derr);
  engine_->process();
  
  double out = assistance_->getValue();
  
  if ( isnan(out) )
  {
    out = max_fl_;
    ROS_WARN_STREAM_THROTTLE(5.0,"setting assistance to max: "<< out);
  }
    
  ROS_INFO_STREAM_THROTTLE(1.0,"given dforce: "<<d_force_->getValue()<<", and derr "<< d_err_->getValue()<<", fl returns assistance = "<< out);
  
  return out;
}




