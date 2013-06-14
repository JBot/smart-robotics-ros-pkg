#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>



//#include <move_base/move_base.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/GetPlan.h>
#include <navfn/navfn.h>
#include <navfn/navfn_ros.h>

#include <pluginlib/class_loader.h>

#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>




#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                                         // for in-/output
#include <string.h>                                        // strcat
#include <fcntl.h>                                         // for 'O_RDONLY' deklaration
#include <termios.h>                                       // for serial

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>

#include <vector>
#include <list>


class TrajectoryManager {
  public:
    TrajectoryManager(tf::TransformListener& tf);
    void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose);
    void recompute_path(void);

    // Goal suscriber
    ros::Subscriber goal_sub_;

    ros::Publisher path_pub;

    ros::Subscriber pathdone_sub_;
    ros::Publisher pathimpossible_pub;
     int cpt_pathimp;
  private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
    void pathDoneCallback(const std_msgs::Empty::ConstPtr & pose);
     ros::NodeHandle nh;

     int status; // 0 = pause ; 1 = checking path
     int cpt;
     nav_msgs::Path my_path;
     geometry_msgs::PoseStamped final_pose;

     tf::TransformListener& tf_;
     costmap_2d::Costmap2DROS* planner_costmap_;
     //nav_core::BaseGlobalPlanner planner_;
     navfn::NavfnROS* planner_;
     //boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
     //pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;


};

TrajectoryManager::TrajectoryManager(tf::TransformListener& tf):
tf_(tf)
{

    status = 0;	
    cpt = 0;
    cpt_pathimp = 0;
    // Goal suscriber
    goal_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("/ROBOT/goal", 20, &TrajectoryManager::goalCallback, this);

    path_pub = nh.advertise < nav_msgs::Path > ("/ROBOT/plan", 50);

    pathdone_sub_ = nh.subscribe < std_msgs::Empty > ("/path_done", 20, &TrajectoryManager::pathDoneCallback, this);
    pathimpossible_pub = nh.advertise < std_msgs::Empty > ("/goal_unreachable", 20);

    my_path.poses = std::vector < geometry_msgs::PoseStamped > ();

    if (my_path.poses.std::vector < geometry_msgs::PoseStamped >::size() >
        (my_path.poses.std::vector < geometry_msgs::PoseStamped >::max_size() - 2)) {
        my_path.poses.std::vector < geometry_msgs::PoseStamped >::pop_back();
    }

    final_pose.pose.position.x = 0.0;
    final_pose.pose.position.y = 0.14;
    final_pose.pose.position.z = 0.0;
    //final_pose.theta = 0.0;

    //tf_ = tf;

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ = new costmap_2d::Costmap2DROS("NESTOR_costmap", tf_);
    planner_costmap_->pause();

    //initialize the global planner
    //bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner");
    planner_ = new navfn::NavfnROS("NESTOR_planner", planner_costmap_);
    //planner->initialize("NESTOR_planner", planner_costmap);

    planner_costmap_->start();    









}

void TrajectoryManager::recompute_path(void)
{
/*
if( (status == 1) ){
   if(cpt > 20) { // 30 // 40 // 50
    nav_msgs::GetPlan tmp_plan;

    common_smart_nav::GetRobotPose tmp_pose;



    if (get_pose.call(tmp_pose))
    {  
            //ROS_INFO("Sum: %ld", get_path.response.plan);
        tmp_plan.request.start = tmp_pose.response.pose;
    }   
    else
    {
            ROS_ERROR("Failed to call service GetRobotPose");
    }

    if( sqrt( pow(final_pose.pose.position.x - tmp_pose.response.pose.pose.position.x, 2) + pow(final_pose.pose.position.y - tmp_pose.response.pose.pose.position.y, 2) ) < 0.22 ) {

	status = 0;

    }
    else {
	    //tmp_plan.request.start = 90;
	    tmp_plan.request.goal = final_pose;
	    tmp_plan.request.tolerance = 0.01;
	    if (get_path.call(tmp_plan))
	    {  

		    //ROS_INFO("Sum: %ld", get_path.response.plan);
		    if(!(tmp_plan.response.plan.poses.std::vector<geometry_msgs::PoseStamped >::empty())) {
			    tmp_plan.response.plan.poses.std::vector<geometry_msgs::PoseStamped >::pop_back();
			    tmp_plan.response.plan.poses.std::vector<geometry_msgs::PoseStamped >::push_back(final_pose);
		    }
		    path_pub.publish(tmp_plan.response.plan);

		    if(tmp_plan.response.plan.poses.std::vector<geometry_msgs::PoseStamped >::empty()) {
			    // Goal unreachable
			    std_msgs::Empty empty;
			    pathimpossible_pub.publish(empty);
			    if(cpt_pathimp == 3) {
			    	status = 0;
				cpt_pathimp = 0;
			    }
			    else  {
				status = 1;
				cpt_pathimp++;
			    }
			    return;
		    }
		    else {
			cpt_pathimp = 0;

		    }

	    }   
	    else
	    {
		    ROS_ERROR("Failed to call service GetPlan");
	    }


	    ROS_INFO("Trajectory manager : Goal RE-sent.");
    }
    cpt = 0;

  }
  else {
    cpt++;
  }
}
*/

}

void TrajectoryManager::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose)
{
    // Assuming the angles are in radians.
    double c1 = cos(heading / 2);
    double s1 = sin(heading / 2);
    double c2 = cos(attitude / 2);

    double s2 = sin(attitude / 2);
    double c3 = cos(bank / 2);
    double s3 = sin(bank / 2);
    double c1c2 = c1 * c2;
    double s1s2 = s1 * s2;

    pose->pose.orientation.w = c1c2 * c3 - s1s2 * s3;
    pose->pose.orientation.x = c1c2 * s3 + s1s2 * c3;
    pose->pose.orientation.y = s1 * c2 * c3 + c1 * s2 * s3;
    pose->pose.orientation.z = c1 * s2 * c3 - s1 * c2 * s3;
}

void TrajectoryManager::pathDoneCallback(const std_msgs::Empty::ConstPtr & pose)
{
  status = 0;
}

void TrajectoryManager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & pose)
{
    final_pose = *pose;
    std::vector<geometry_msgs::PoseStamped> global_plan;
    nav_msgs::Path tmp_path;


    //make sure we have a costmap for our planner
    if(planner_costmap_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      //return false;
    }

    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_->getRobotPose(global_pose)){
      ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
      //return false;
    }

    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    //if(req.start.header.frame_id == "")
      tf::poseStampedTFToMsg(global_pose, start);



	if(planner_->makePlan(start, final_pose, global_plan)){
          if(!global_plan.empty()){
            global_plan.push_back(final_pose);
	  }
 	}

    tmp_path.poses = global_plan;

    path_pub.publish(tmp_path);


/*
    common_smart_nav::GetRobotPose tmp_pose;

    if (get_pose.call(tmp_pose))
    {
            //ROS_INFO("Sum: %ld", get_path.response.plan);
        tmp_plan.request.start = tmp_pose.response.pose;
    }
    else
    {
            ROS_ERROR("Failed to call service GetRobotPose");
    }


    //tmp_plan.request.start = 90;
    tmp_plan.request.goal = *pose;
    tmp_plan.request.tolerance = 0.01;
    if (get_path.call(tmp_plan))
    {
	    //ROS_INFO("Sum: %ld", get_path.response.plan);
        if(!(tmp_plan.response.plan.poses.std::vector<geometry_msgs::PoseStamped >::empty())) {
            tmp_plan.response.plan.poses.std::vector<geometry_msgs::PoseStamped >::pop_back();
            tmp_plan.response.plan.poses.std::vector<geometry_msgs::PoseStamped >::push_back(final_pose);
        }
	path_pub.publish(tmp_plan.response.plan);
	
	if(tmp_plan.response.plan.poses.std::vector<geometry_msgs::PoseStamped >::empty()) {
		// Goal unreachable
		std_msgs::Empty empty;
		pathimpossible_pub.publish(empty);

		return;
	}
	else {

	}
    }
    else
    {
            ROS_ERROR("Failed to call service GetPlan");
    }


    ROS_INFO("Trajectory manager : Path sent.");

    status = 1;
    cpt = 0;
*/
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{


        /**
         * The ros::init() function needs to see argc and argv so that it can perform
         * any ROS arguments and name remapping that were provided at the command line. For programmatic
         * remappings you can use a different version of init() which takes remappings
         * directly, but for most command-line programs, passing argc and argv is the easiest
         * way to do it.  The third argument to init() is the name of the node.
         *
         * You must call one of the versions of ros::init() before using any other
         * part of the ROS system.
         */
        ros::init(argc, argv, "Trajectory_Manager");
        tf::TransformListener listener(ros::Duration(10));
        TrajectoryManager trajectorymanager(listener);

	ros::spin();

        // Refresh rate
/*        ros::Rate loop_rate(50);
        float rotation = 0.0;
        while (ros::ok()) {

                ros::spinOnce();
                loop_rate.sleep();
		trajectorymanager.recompute_path();
        }

        ros::Duration(2.0).sleep();
*/

        ros::shutdown();
}



