#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <trajectory_plotter/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <rosdyn_core/primitives.h>

#include <thread>
#include <moveit_planning_helper/manage_trajectories.h>

#include <std_msgs/Float32.h>

// class robotTrajPlanner{
// public:
// 
//   void bc()
//   {
//     static tf::TransformBroadcaster br;
//     while (ros::ok())
//     {
//       br.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "base_link", "target_pose"));
//       ros::Duration(0.01).sleep();
//     }
//   }
// 
//   void setTf(tf::Pose& tf){tf_ = tf;}
//   
// private:
//   tf::Pose tf_;
// 
// };



tf::Pose tf_;

double distance_ = 10;

void bc()
{
  static tf::TransformBroadcaster br;
  while (ros::ok())
  {
    br.sendTransform(tf::StampedTransform(tf_, ros::Time::now(), "base_link", "target_pose"));
    ros::Duration(0.01).sleep();
  }
}

void callback(const std_msgs::Float32::ConstPtr& msg)
{
  distance_ = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO_STREAM("[mqtt converter] -> looking for params under NS: "<< nh.getNamespace());
  
  std::vector<double> target_vec,home_vec;
  if(!nh.getParam("target",target_vec))
  {
    ROS_ERROR_STREAM("target not found on namespace: "<<nh.getNamespace());
    return -1;
  }
  if(!nh.getParam("home",home_vec))
  {
    ROS_ERROR_STREAM("home not found on namespace: "<<nh.getNamespace());
    return -1;
  }
  int rate;
  GET_AND_DEFAULT(nh,"rate",rate,10);

  std::string base_link, tool_link, action_name;
  GET_AND_RETURN(nh,"action_name",action_name);
  GET_AND_RETURN(nh,"base_link",base_link);
  GET_AND_RETURN(nh,"tool_link",tool_link);
  bool replan;
  GET_AND_RETURN(nh,"replan",replan);
  
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
  {
    ROS_ERROR("Urdf robot_description '%s' does not exist",(nh.getNamespace()+"/robot_description").c_str());
  }
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;
  rosdyn::ChainPtr chain_bt = rosdyn::createChain(urdf_model, base_link, tool_link, gravity);
  
  ros::Rate r = rate;
  
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execute_trajectory ( action_name, true );
  ROS_INFO_STREAM("waitin for server "<<action_name);
  execute_trajectory.waitForServer();
  ROS_INFO_STREAM(action_name<<" connected ! ");
  
  
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  std::vector<std::vector<double>> poses_vec;
  
  poses_vec.push_back(target_vec);
  poses_vec.push_back(home_vec);
  
//   robotTrajPlanner rtp;
  
  
  std::thread broadcast(bc);
//   std::thread broadcast_thread(&robotTrajPlanner::bc, rtp);
  {
    Eigen::VectorXd vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(poses_vec[0].data(), poses_vec[0].size());
    Eigen::Affine3d T_bt = chain_bt->getTransformation(vec);
    tf::Pose tf;
    tf::poseEigenToTF (T_bt, tf);
    tf::poseEigenToTF (T_bt, tf_);
//     rtp.setTf(tf);
  }
  
  
  std::vector<std::string> trj_names;
  for (size_t i=0; i<poses_vec.size();i++)
    trj_names.push_back("traj_"+std::to_string(i));

  ros::Subscriber sub = nh.subscribe("/closeness", 1000, callback);
  
  for (size_t i=0; i<poses_vec.size();i++)
  {
    
    if(replan)
    {
      move_group.setStartStateToCurrentState();
      move_group.setJointValueTarget(poses_vec.at(i));
      move_group.plan(my_plan);
      
      std::string path = "/trajectories_planned/"+trj_names.at(i);
      ROS_INFO_STREAM("saving trajectory " << path << " to param");
      std::string what;
      if(! trajectory_processing::setTrajectoryToParam(path, my_plan.trajectory_.joint_trajectory,what))
        ROS_ERROR(" not set on ros param");
    }
    else
    {
      std::string path = "/trajectories_planned/"+trj_names.at(i);
      ROS_INFO_STREAM("recovering trajectory " << path << " from param");
      std::string what;
      if(! trajectory_processing::getTrajectoryFromParam(path,my_plan.trajectory_.joint_trajectory,what) )
        ROS_ERROR_STREAM("trajectory " << path << " not set on ros param. what : " << what);        
    }
    
    ROS_INFO_STREAM(CYAN<<"Press enter to start motion");
    std::cin.get(); 
    
    Eigen::VectorXd vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(poses_vec.at(i).data(), poses_vec.at(i).size());
    Eigen::Affine3d T_bt = chain_bt->getTransformation(vec);
    
    tf::Pose tf;
    tf::poseEigenToTF (T_bt, tf);
    tf::poseEigenToTF (T_bt, tf_);
//     rtp.setTf(tf);
/*
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM_THROTTLE(5.0,"looping");
    
    control_msgs::FollowJointTrajectoryGoal g;
    
    g.trajectory = my_plan.trajectory_.joint_trajectory;
    
    execute_trajectory.sendGoal ( g );
    
    distance_ = 10;
    
    std::vector<double> first_point;
    ROS_INFO_STREAM("first traj point");
    for (auto p : g.trajectory.points[0].positions)
      ROS_INFO_STREAM(p);
    
    ROS_INFO_STREAM("last traj point");      
    for (auto p : g.trajectory.points.back().positions)
      ROS_INFO_STREAM(p);
    
    ROS_INFO_STREAM("waitin for execution");
    
    actionlib::SimpleClientGoalState as = execute_trajectory.getState();
    
    while(as != actionlib::SimpleClientGoalState::SUCCEEDED )
    {
      
//       br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "base_link", "target_pose"));
      
      as = execute_trajectory.getState();
      
      if(as == actionlib::SimpleClientGoalState::ACTIVE) 
        ROS_INFO_STREAM_THROTTLE(2.0,GREEN<<"executing trajectory. State :  ACTIVE !" );
      else if(as == actionlib::SimpleClientGoalState::SUCCEEDED) 
        ROS_INFO_STREAM(RED<<"executing trajectory. State :  SUCCEEDED !" );
      else if(as == actionlib::SimpleClientGoalState::ABORTED) 
      {
        ROS_INFO_STREAM(YELLOW<<"executing trajectory. State :  ABORTED !" );
        ROS_ERROR("Trajectory not completely executed . Aborting");
        break;
      }
      else if(as == actionlib::SimpleClientGoalState::LOST) 
      {
        ROS_INFO_STREAM(BLUE<<"executing trajectory. State :  LOST !" );
        break;
      }
      else if(as == actionlib::SimpleClientGoalState::PENDING) 
        ROS_INFO_STREAM(MAGENTA<<"executing trajectory. State :  PENDING !" );
      else if(as == actionlib::SimpleClientGoalState::RECALLED) 
      {
        ROS_INFO_STREAM(CYAN<<"executing trajectory. State :  RECALLED !" );
        break;
      }
      else if(as == actionlib::SimpleClientGoalState::REJECTED)
      {
        ROS_INFO_STREAM(WHITE<<"executing trajectory. State :  REJECTED!" );
        break;
      }
      else if(as == actionlib::SimpleClientGoalState::PREEMPTED)
      {
        ROS_INFO_STREAM(CYAN<<"executing trajectory. State :  PREEMPTED!" );
        break;
      }
      
      if(distance_<0.005)
      {
        execute_trajectory.cancelAllGoals();
        ROS_INFO_STREAM(GREEN<<"position in tolerance reaced . Stopping");
        break;
      }
      
      r.sleep(); 
    }
    if ( !execute_trajectory.getResult() )
    {
      ROS_ERROR("some error in trajectory execution. Return!");
      return -1;
    }*/
    ROS_INFO_STREAM(GREEN << "Trajectory executed correctly ! ");
    
  }
  
  broadcast.join();
//   broadcast_thread.join();
  

  return 0;
}

