#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <joy/Joy.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                               // for in-/output
#include <string.h>                              // strcat
#include <fcntl.h>                               // for 'O_RDONLY' deklaration
#include <termios.h>                             // for serial

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>

#include <vector>

#define BAUDRATE B115200

class TeleopMaximus
{
	public:
		TeleopMaximus();
		void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped *pose);
		void publish_all(void);
		void get_value_and_do_computation(void);

		// Joystick suscriber
		ros::Subscriber joy_sub_;
		// Goal suscriber
		ros::Subscriber goal_sub_;
		// Set the position of the robot
		ros::Subscriber pose_in_map_sub_;
		// Set the path of the robot => Doesn't work ...
		ros::Publisher path_in_map_pub;


	private:
		void joyCallback(const joy::Joy::ConstPtr& joy);
		void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
		void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
		ros::NodeHandle nh;

		int i;

		int linear_port, angular_port;
		double l_scale_, a_scale_;
		double linear_value, angular_value, prev_linear_value, prev_angular_value;
		float heading;

		tf::TransformBroadcaster br;
		tf::Transform transform;
		nav_msgs::Path my_maximus_path;
		geometry_msgs::PoseStamped temp_pose;

};

TeleopMaximus::TeleopMaximus():
	linear_port(1),
	angular_port(2),
	a_scale_(12000),
	l_scale_(12000)
{

	nh.param("axis_linear", linear_port, linear_port);
	nh.param("axis_angular", angular_port, angular_port);
	nh.param("scale_angular", a_scale_, a_scale_);
	nh.param("scale_linear", l_scale_, l_scale_);

	linear_value = 0;
	angular_value = 0;
	prev_linear_value = 0;
	prev_angular_value = 0;

	heading = 0;

	// Joystick suscriber
	joy_sub_ = nh.subscribe<joy::Joy>("joy", 10, &TeleopMaximus::joyCallback, this);
	// Goal suscriber
	goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("maximus_goal", 10, &TeleopMaximus::goalCallback, this);
	// Set the position of the robot
	pose_in_map_sub_  = nh.subscribe<geometry_msgs::PoseStamped>("avr_maximus_pose", 20, &TeleopMaximus::poseCallback, this);
	// Set the path of the robot 
	path_in_map_pub  = nh.advertise<nav_msgs::Path>("maximus_path", 50);


	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/between_wheels"));

	

	my_maximus_path.header.frame_id = "/map";
	my_maximus_path.header.stamp = ros::Time::now();

	my_maximus_path.poses = std::vector<geometry_msgs::PoseStamped>();

	if(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::size() > (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::max_size()-2)) {
		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::pop_back();
	}


	i = 0;

}


void TeleopMaximus::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped *pose) {
	// Assuming the angles are in radians.
	double c1 = cos(heading/2);
	double s1 = sin(heading/2);
	double c2 = cos(attitude/2);

	double s2 = sin(attitude/2);
	double c3 = cos(bank/2);
	double s3 = sin(bank/2);
	double c1c2 = c1*c2;
	double s1s2 = s1*s2;

	pose->pose.orientation.w = c1c2*c3 - s1s2*s3;
	pose->pose.orientation.x = c1c2*s3 + s1s2*c3;
	pose->pose.orientation.y = s1*c2*c3 + c1*s2*s3;
	pose->pose.orientation.z = c1*s2*c3 - s1*c2*s3;
}

void TeleopMaximus::joyCallback(const joy::Joy::ConstPtr& joy)
{
	char Serout[260]={0};
	angular_value = a_scale_*joy->axes[angular_port];
	linear_value = l_scale_*joy->axes[linear_port];
/*
	if(ser_fd != -1) {
		sprintf(Serout, "T000000");
		write(ser_fd, &Serout, 7);
		ROS_INFO("GOFRONT %s", Serout);
	}
*/
}

void TeleopMaximus::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
	char Serout[260]={0};
	double x, y;
	char byte1, byte2, byte3, byte4, byte5, byte6;
	x = abs(goal->pose.position.x*1000*2/10);
	y = abs(goal->pose.position.y*1000*2/10);

}

void TeleopMaximus::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{

temp_pose.pose.position.x = pose->pose.position.x;
temp_pose.pose.position.y = pose->pose.position.y;
temp_pose.pose.orientation.x = pose->pose.orientation.x;
temp_pose.pose.orientation.y = pose->pose.orientation.y;
temp_pose.pose.orientation.z = pose->pose.orientation.z;
temp_pose.pose.orientation.w = pose->pose.orientation.w;


        my_maximus_path.header.stamp = ros::Time::now();
        if(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::size() > (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::max_size()-2)) {
                my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::pop_back();
        }
        my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped>::push_back(*pose);

        transform.setOrigin( tf::Vector3(pose->pose.position.x, pose->pose.position.y, 0.0) );
	transform.setRotation( tf::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w) );
        //br.sendTransform(tf::StampedTransform(transform, pose->header.stamp, "/map", "/between_wheels"));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/between_wheels"));

        i = i + 5;
        if( i > 180)
                i = -180;
        ROS_INFO("X=%f /Y=%f /T=%f /L=%f", pose->pose.position.x, pose->pose.position.y, pose->pose.orientation.z*180/3.1415, linear_value);

	TeleopMaximus::path_in_map_pub.publish(my_maximus_path);

}


void TeleopMaximus::get_value_and_do_computation(void) {

}


void TeleopMaximus::publish_all(void) {
	// Publish Path of the robot
	//TeleopMaximus::path_in_map_pub.publish(my_maximus_path);

TeleopMaximus::transform.setOrigin( tf::Vector3(TeleopMaximus::temp_pose.pose.position.x, TeleopMaximus::temp_pose.pose.position.y, 0.0) );
TeleopMaximus::transform.setRotation( tf::Quaternion(TeleopMaximus::temp_pose.pose.orientation.x, TeleopMaximus::temp_pose.pose.orientation.y, TeleopMaximus::temp_pose.pose.orientation.z, TeleopMaximus::temp_pose.pose.orientation.w) );
TeleopMaximus::br.sendTransform(tf::StampedTransform(TeleopMaximus::transform, ros::Time::now(), "/map", "/between_wheels"));



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
		ros::init(argc, argv, "maximus_position");
		TeleopMaximus maximus_talker;
		// Refresh rate
		ros::Rate loop_rate(5); // 35 with bluetooth
		float rotation = 0.0;
		while (ros::ok())
		{
			// Get the values and do the computation
			//maximus_talker.get_value_and_do_computation();
			// Publish all the values and messages
			//maximus_talker.publish_all();


//			TeleopMaximus::transform.setOrigin( tf::Vector3(TeleopMaximus::temp_pose.pose.position.x, TeleopMaximus::temp_pose.pose.position.y, 0.0) );
//		        TeleopMaximus::transform.setRotation( tf::Quaternion(TeleopMaximus::temp_pose.pose.orientation.x, TeleopMaximus::temp_pose.pose.orientation.y, TeleopMaximus::temp_pose.pose.orientation.z, TeleopMaximus::temp_pose.pose.orientation.w) );
//        		TeleopMaximus::br.sendTransform(tf::StampedTransform(TeleopMaximus::transform, ros::Time::now(), "/map", "/between_wheels"));


			ros::spinOnce();
			loop_rate.sleep();
		}

		ros::Duration(2.0).sleep();

		ros::shutdown();
	}




