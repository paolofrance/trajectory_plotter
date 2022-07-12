#include <trajectory_plotter/human_traj_estimation.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_estimation_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  TrajEstimator te(nh);
  
  std::string wrench_topic, vel_topic, assistance_topic;
  
  if ( !nh.getParam ( "wrench_topic", wrench_topic) )
  {
    wrench_topic = "filtered_wrench_base";
    ROS_WARN_STREAM (nh.getNamespace() << " /wrench_topic not set. default " << wrench_topic );
  }
    if ( !nh.getParam ( "vel_topic", vel_topic) )
  {
    vel_topic = "current_velocity";
    ROS_WARN_STREAM (nh.getNamespace() << " /vel_topic not set. default "<< vel_topic);
  }
    if ( !nh.getParam ( "assistance_topic", assistance_topic) )
  {
    assistance_topic = "assistance";
    ROS_WARN_STREAM (nh.getNamespace() << " /assistance_topic not set. default " << assistance_topic);
  }
  
  ros::Subscriber wrench_sub   = nh.subscribe(wrench_topic, 10, &TrajEstimator::wrenchCallback  , &te);
  ros::Subscriber velocity_sub = nh.subscribe(vel_topic   , 10, &TrajEstimator::velocityCallback, &te);
  
  ros::Publisher assistance_pub = nh.advertise<geometry_msgs::TwistStamped>(assistance_topic,10);
  
  ros::Duration(0.1).sleep();
  
  ros::Rate rate(125);
  
  while (ros::ok())
  {
    Eigen::Vector6d der = te.getDerr();
    Eigen::Vector6d dw = te.getDwrench();
    
    double al_x = te.evaluateFis(der(0),dw(0));
    double al_y = te.evaluateFis(der(1),dw(1));
    double al_z = te.evaluateFis(der(2),dw(2));
    
    geometry_msgs::TwistStamped msg;
    
    msg.twist.linear.x = al_x;
    msg.twist.linear.y = al_y;
    msg.twist.linear.z = al_z;
    msg.twist.angular.x = 0;
    msg.twist.angular.y = 0;
    msg.twist.angular.z = 0;
    
    msg.header.stamp = ros::Time::now();
    
    assistance_pub.publish(msg);
    
    rate.sleep();
  }
  ros::waitForShutdown();
  
  return 0;
  
}

