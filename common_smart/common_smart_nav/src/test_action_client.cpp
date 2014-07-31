#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <common_smart_nav/move_robotAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_move_robot");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<common_smart_nav::move_robotAction> ac("PETIT_pathplanner", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  common_smart_nav::move_robotGoal goal;
  goal.goal.header.frame_id = "/petit_map"; 
  goal.goal.header.stamp = ros::Time::now();
  goal.goal.pose.position.x = 1.0;
  goal.goal.pose.position.y = 1.0;
  goal.goal.pose.position.z = 0.0;
  goal.goal.pose.orientation.x = 0.0;
  goal.goal.pose.orientation.y = 0.0;
  goal.goal.pose.orientation.z = 0.0;
  goal.goal.pose.orientation.w = 1.0;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
