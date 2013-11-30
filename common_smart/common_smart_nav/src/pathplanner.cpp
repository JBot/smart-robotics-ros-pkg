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

#include "common_smart_nav/GetRobotPose.h"
#include "common_smart_nav/GetPlan.h"
#include "common_smart_nav/GetDistance.h"

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


#define STOP 	0
#define PAUSE	1
#define RUN	2


class TrajectoryManager {
	public:
		TrajectoryManager(tf::TransformListener& tf);
		~TrajectoryManager();
		void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose);
		void recompute_path(void);

		// Goal suscriber
		ros::Subscriber goal_sub_;
		ros::Subscriber pathdone_sub_;
		ros::Subscriber computepath_sub_;
		ros::Subscriber pause_sub_;
		ros::Subscriber resume_sub_;

		ros::Publisher path_pub;
		ros::Publisher pathimpossible_pub;

		ros::ServiceServer pose_service;
		ros::ServiceServer distance_service;
		ros::ServiceServer path_service;

		int cpt_pathimp;
	private:
		void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
		void pathDoneCallback(const std_msgs::Empty::ConstPtr & pose);
		void computePathCallback(const std_msgs::Empty::ConstPtr & pose);
		void pauseCallback(const std_msgs::Empty::ConstPtr & pose);
		void resumeCallback(const std_msgs::Empty::ConstPtr & pose);

		void computePath(void);
		void planThread(void);
		void publishPath(void);

		bool getRobotPose(common_smart_nav::GetRobotPose::Request  &req, common_smart_nav::GetRobotPose::Response &res );
		bool getPath(common_smart_nav::GetPlan::Request  &req, common_smart_nav::GetPlan::Response &res );
		bool getDistance(common_smart_nav::GetDistance::Request  &req, common_smart_nav::GetDistance::Response &res );

		double compute_distance(nav_msgs::Path path_to_compute);


		ros::NodeHandle nh;

		int status; // STOP PAUSE RUN
		int cpt;

		nav_msgs::Path my_path;
		geometry_msgs::PoseStamped final_pose;
		geometry_msgs::PoseStamped current_pose;

		boost::thread* planner_thread_;

		tf::TransformListener& tf_;
		costmap_2d::Costmap2DROS* planner_costmap_;
		//nav_core::BaseGlobalPlanner planner_;
		navfn::NavfnROS* planner_;
		//boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
		//pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;

		std::string map_name;

};

TrajectoryManager::TrajectoryManager(tf::TransformListener& tf):
	tf_(tf)
{

	std::string costmap_name;
	std::string planner_name;
	ros::NodeHandle nhp("~");

	//nh.param<std::string>("costmap_name", costmap_name, "ROBOT_costmap");
	//nh.param<std::string>("planner_name", planner_name, "ROBOT_planner");
	nhp.getParam("costmap_name", costmap_name);
	nhp.getParam("planner_name", planner_name);
	nhp.getParam("map_name", map_name);

	status = 0;	
	cpt = 0;
	cpt_pathimp = 0;
	// Goal suscriber
	goal_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("/ROBOT/goal", 2, &TrajectoryManager::goalCallback, this);

	path_pub = nh.advertise < nav_msgs::Path > ("/ROBOT/plan", 5);

	pathdone_sub_ = nh.subscribe < std_msgs::Empty > ("/ROBOT/path_done", 2, &TrajectoryManager::pathDoneCallback, this);
	computepath_sub_ = nh.subscribe < std_msgs::Empty > ("/ROBOT/compute_path", 2, &TrajectoryManager::computePathCallback, this);
	pause_sub_ = nh.subscribe < std_msgs::Empty > ("/ROBOT/pause", 2, &TrajectoryManager::pauseCallback, this);
	resume_sub_ = nh.subscribe < std_msgs::Empty > ("/ROBOT/resume", 2, &TrajectoryManager::resumeCallback, this);

	pathimpossible_pub = nh.advertise < std_msgs::Empty > ("/goal_unreachable", 2);



	pose_service = nh.advertiseService("/ROBOT/get_robot_pose", &TrajectoryManager::getRobotPose, this);
	path_service = nh.advertiseService("/ROBOT/get_path", &TrajectoryManager::getPath, this);
	distance_service = nh.advertiseService("/ROBOT/get_distance", &TrajectoryManager::getDistance, this);


	//just an arbitrary point in space
	current_pose.header.frame_id = map_name;
	current_pose.header.stamp = ros::Time();
	current_pose.pose.position.x = 0.0;
	current_pose.pose.position.y = 0.0;
	current_pose.pose.position.z = 0.0;

	current_pose.pose.orientation.x = 0.0;
	current_pose.pose.orientation.y = 0.0;
	current_pose.pose.orientation.z = 0.0;
	current_pose.pose.orientation.w = 1.0;




	my_path.poses = std::vector < geometry_msgs::PoseStamped > ();

	if (my_path.poses.std::vector < geometry_msgs::PoseStamped >::size() >
			(my_path.poses.std::vector < geometry_msgs::PoseStamped >::max_size() - 2)) {
		my_path.poses.std::vector < geometry_msgs::PoseStamped >::pop_back();
	}

	my_path.header.frame_id = map_name;

	final_pose.pose.position.x = 0.0;
	final_pose.pose.position.y = 0.14;
	final_pose.pose.position.z = 0.0;
	//final_pose.theta = 0.0;

	//tf_ = tf;

	//create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
	planner_costmap_ = new costmap_2d::Costmap2DROS(costmap_name, tf_);
	planner_costmap_->pause();

	//initialize the global planner
	//bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner");
	planner_ = new navfn::NavfnROS(planner_name, planner_costmap_);
	//planner->initialize("NESTOR_planner", planner_costmap);

	planner_costmap_->start();    

	status = PAUSE;


	planner_thread_ = new boost::thread(boost::bind(&TrajectoryManager::planThread, this));




}

TrajectoryManager::~TrajectoryManager()
{

	planner_thread_->interrupt();
	planner_thread_->join();

}

void TrajectoryManager::recompute_path(void)
{

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

double TrajectoryManager::compute_distance(nav_msgs::Path path_to_compute)
{
	double distance = 0.0;
	geometry_msgs::Pose2D current_pose;

	if ( !(path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::empty()) ) {
		current_pose.x = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x;
		current_pose.y = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y;
		path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::erase (path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::begin());

		//ROS_ERROR("current X : %f, X : %f / current Y : %f, Y : %f", current_pose.x, path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, current_pose.y, path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y);

		while ( !(path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::empty()) ) {
			distance += sqrt( pow(current_pose.x - path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, 2) + pow(current_pose.y - path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y, 2) );

			current_pose.x = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x;
			current_pose.y = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y;
			path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::erase (path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::begin());

			//ROS_ERROR("current X : %f, X : %f / current Y : %f, Y : %f", current_pose.x, path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, current_pose.y, path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y);
		}

	}
	//ROS_ERROR("Distance : %f", distance);

	return distance;

}



bool TrajectoryManager::getRobotPose(common_smart_nav::GetRobotPose::Request  &req,
		common_smart_nav::GetRobotPose::Response &res )
{
	res.pose = current_pose;
	//res.sum = req.a + req.b;
	//ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	//ROS_INFO("sending back response: [%ld]", (long int)res.sum);
	return true;
}

bool TrajectoryManager::getPath(common_smart_nav::GetPlan::Request  &req,
		common_smart_nav::GetPlan::Response &res )
{
	//res.sum = req.a + req.b;
	//ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
	//ROS_INFO("sending back response: [%ld]", (long int)res.sum);

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

	current_pose = start;

	if(planner_->makePlan(start, req.goal, global_plan)){
		//ROS_ERROR("planner makes plan");
		if(!global_plan.empty()){
			global_plan.push_back(req.goal);
			//ROS_ERROR("globalplan filled");
		}
	}

	tmp_path.header.frame_id = map_name;
	tmp_path.poses = global_plan;

	res.plan = tmp_path;



	return true;
}

bool TrajectoryManager::getDistance(common_smart_nav::GetDistance::Request  &req,
		common_smart_nav::GetDistance::Response &res )
{

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

	current_pose = start;

	if(planner_->makePlan(start, req.goal, global_plan)){
		//ROS_ERROR("planner makes plan");
		if(!global_plan.empty()){
			global_plan.push_back(req.goal);
			//ROS_ERROR("globalplan filled");
		}
	}

	tmp_path.header.frame_id = map_name;
	tmp_path.poses = global_plan;

	res.distance.data = compute_distance(tmp_path);

	return true;
}



void TrajectoryManager::pathDoneCallback(const std_msgs::Empty::ConstPtr & pose)
{
	status = PAUSE;
}

void TrajectoryManager::pauseCallback(const std_msgs::Empty::ConstPtr & pose)
{
	status = STOP;
	planner_costmap_->pause();
}

void TrajectoryManager::resumeCallback(const std_msgs::Empty::ConstPtr & pose)
{
	status = PAUSE;
	planner_costmap_->resume();
}

void TrajectoryManager::computePathCallback(const std_msgs::Empty::ConstPtr & pose)
{

	computePath();
	publishPath();
	status = RUN;
}

void TrajectoryManager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & pose)
{
	final_pose = *pose;

	computePath();
	publishPath();

	status = RUN;
}

void TrajectoryManager::computePath(void)
{
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

	current_pose = start;

	if(planner_->makePlan(start, final_pose, global_plan)){
		//ROS_ERROR("planner makes plan");
		if(!global_plan.empty()){
			global_plan.push_back(final_pose);
			//ROS_ERROR("globalplan filled");
		}
	}

	tmp_path.header.frame_id = map_name;
	tmp_path.poses = global_plan;

	// lock
	my_path = tmp_path;
	// unlock

}

void TrajectoryManager::publishPath(void)
{
	path_pub.publish(my_path);
}

void TrajectoryManager::planThread(void)
{
	ros::Rate r(20);
	while(nh.ok()) {

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

		//ROS_ERROR("PathPlanner : Compute current pose");
		current_pose = start;


		switch(status) {
			case STOP:
				break;
			case PAUSE:
				break;
			case RUN:
				//computePath();
				//publishPath();
				break;	
			default:
				break;
		}
		r.sleep();
	}

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

	ros::shutdown();
}



