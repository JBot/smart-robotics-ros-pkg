#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "maximus_navigation_goals");

  tf::TransformListener listener(ros::Duration(10));

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);


  geometry_msgs::PoseStamped odom_pose;
  geometry_msgs::PoseStamped base_pose;
  odom_pose.header.frame_id = "/base_link";

  //we'll just use the most recent transform available for our simple example
  odom_pose.header.stamp = ros::Time();
  
  //just an arbitrary point in space
  odom_pose.pose.position.x = 0.0;
  odom_pose.pose.position.y = 0.0;
  odom_pose.pose.position.z = 0.0;
    
  odom_pose.pose.orientation.x = 0.0;
  odom_pose.pose.orientation.y = 0.0;
  odom_pose.pose.orientation.z = 0.0;
  odom_pose.pose.orientation.w = 1.0;

  try{  
          ros::Time now = ros::Time::now();
          listener.waitForTransform("/odom", "/base_link", now, ros::Duration(5.0));
          listener.transformPose("/odom", odom_pose, base_pose);



  }
  catch(tf::TransformException& ex){
          ROS_ERROR("Received an exception trying to transform a point from \"odom\" to \"base_link\": %s", ex.what());
  }       

while( sqrt( pow(goal.target_pose.pose.position.x - base_pose.pose.position.x, 2) + pow(goal.target_pose.pose.position.y - base_pose.pose.position.y, 2) ) > 0.05 ) {
  //we'll just use the most recent transform available for our simple example
  odom_pose.header.stamp = ros::Time();

  //just an arbitrary point in space
  odom_pose.pose.position.x = 0.0;
  odom_pose.pose.position.y = 0.0;
  odom_pose.pose.position.z = 0.0;

  odom_pose.pose.orientation.x = 0.0;
  odom_pose.pose.orientation.y = 0.0;
  odom_pose.pose.orientation.z = 0.0;
  odom_pose.pose.orientation.w = 1.0;

  try{
	  ros::Time now = ros::Time::now();
	  listener.waitForTransform("/odom", "/base_link", now, ros::Duration(5.0));
	  listener.transformPose("/odom", odom_pose, base_pose);

  }
  catch(tf::TransformException& ex){
	  ROS_ERROR("Received an exception trying to transform a point from \"odom\" to \"base_link\": %s", ex.what());
	  break;
  }
  ros::Duration(1.0).sleep();
  ac.sendGoal(goal);
}


    ROS_INFO("Hooray, I have reach my goal !");
/*
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
*/
  return 0;
}

