#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>


#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                                         // for in-/output
#include <string.h>                                        // strcat

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>
#include <vector>

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

#define YELLOW_START_X 1.300
#define YELLOW_START_Y 1.850
#define YELLOW_START_Z 1.5707963

#define RED_START_X -1.300
#define RED_START_Y 1.850
#define RED_START_Z 1.5707963


class InitPose {
	public:
		InitPose(tf::TransformListener& tf);

		ros::Subscriber pose_sub;
		ros::Publisher cmd_vel_pub;
		ros::Publisher start_path_pub;

		void broadcast(void);

	private:
		void poseCallback(const std_msgs::Empty::ConstPtr& vel);

		ros::NodeHandle nh;

		tf::TransformListener& tf_;
		//geometry_msgs::TransformStamped t;
		tf::StampedTransform t;
		tf::TransformBroadcaster broadcaster;

		std::string color_name;

		int8_t init_done;

};

InitPose::InitPose(tf::TransformListener& tf) :
	tf_(tf)
{
	geometry_msgs::Twist final_cmd_vel;

	ros::NodeHandle nhp("~");

	nhp.getParam("color_name", color_name);
	ROS_ERROR("Color is : %s", color_name.c_str());

	pose_sub = nh.subscribe<std_msgs::Empty>("ROBOT/init_pose", 5, &InitPose::poseCallback, this);
	cmd_vel_pub = nh.advertise < geometry_msgs::Twist > ("/ROBOT/cmd_vel", 5);
	start_path_pub = nh.advertise < std_msgs::Empty > ("/ROBOT/resume_pathwrapper", 5);


	init_done = 0;

	final_cmd_vel.linear.x = 0;
	final_cmd_vel.linear.y = 0;
	final_cmd_vel.linear.z = 0;
	final_cmd_vel.angular.x = 0;
	final_cmd_vel.angular.y = 0;
	final_cmd_vel.angular.z = 0;

	cmd_vel_pub.publish(final_cmd_vel);

	usleep(500000);



        // Go forward to build the map 
        final_cmd_vel.linear.x = 0.02;
        final_cmd_vel.linear.y = 0;
        final_cmd_vel.angular.z = 0;
        cmd_vel_pub.publish(final_cmd_vel);

        usleep(5000000);

        // Go backward to build the map 
        final_cmd_vel.linear.x = -0.02;
        final_cmd_vel.linear.y = 0;
        final_cmd_vel.angular.z = 0;
        cmd_vel_pub.publish(final_cmd_vel);

        usleep(5000000);



        // Turn on himself to build the map 
        final_cmd_vel.linear.x = 0;
        final_cmd_vel.linear.y = 0;
        final_cmd_vel.angular.z = 0.15;
        cmd_vel_pub.publish(final_cmd_vel);

	usleep(6500000);


	// Turn on himself to build the map 
	final_cmd_vel.linear.x = 0;
	final_cmd_vel.linear.y = 0;
	final_cmd_vel.angular.z = 0.15;
	cmd_vel_pub.publish(final_cmd_vel);

        usleep(15000000);

        // Turn on himself to build the map 
        final_cmd_vel.linear.x = 0;
        final_cmd_vel.linear.y = 0;
        final_cmd_vel.angular.z = -0.15;
        cmd_vel_pub.publish(final_cmd_vel);

	usleep(15000000);

	// Go forward to build the map 
	final_cmd_vel.linear.x = -0.02;
	final_cmd_vel.linear.y = 0;
	final_cmd_vel.angular.z = 0;
	cmd_vel_pub.publish(final_cmd_vel);

	usleep(5000000);

	// Go backward to build the map 
	final_cmd_vel.linear.x = 0.02;
	final_cmd_vel.linear.y = 0;
	final_cmd_vel.angular.z = 0;
	cmd_vel_pub.publish(final_cmd_vel);

	usleep(15000000);

	if(color_name == "Yellow") {
		// Go on the left to join the starting zone 
		final_cmd_vel.linear.x = 0;
		final_cmd_vel.linear.y = -0.02;
		final_cmd_vel.angular.z = 0;
	}
	else {
		// Go on the right to join the starting zone 
		final_cmd_vel.linear.x = 0;
		final_cmd_vel.linear.y = 0.02;
		final_cmd_vel.angular.z = 0;
	}
	cmd_vel_pub.publish(final_cmd_vel);

	usleep(7000000);

        // Go backward to join the starting zone 
        final_cmd_vel.linear.x = 0.02;
        final_cmd_vel.linear.y = 0;
        final_cmd_vel.angular.z = 0;
        cmd_vel_pub.publish(final_cmd_vel);

        usleep(7000000);

	// Stop and wait the init pose signal 
	final_cmd_vel.linear.x = 0;
	final_cmd_vel.linear.y = 0;
	final_cmd_vel.angular.z = 0;
	cmd_vel_pub.publish(final_cmd_vel);

	usleep(5);




}


void InitPose::poseCallback(const std_msgs::Empty::ConstPtr& vel)
{

	geometry_msgs::PoseStamped odom_pose;
	odom_pose.header.frame_id = "/petit_base_link";

	//we'll just use the most recent transform available for our simple example
	odom_pose.header.stamp = ros::Time::now();

	//just an arbitrary point in space
	odom_pose.pose.position.x = 0.0;
	odom_pose.pose.position.y = 0.0;
	odom_pose.pose.position.z = 0.0;

	odom_pose.pose.orientation.x = 0.0;
	odom_pose.pose.orientation.y = 0.0;
	odom_pose.pose.orientation.z = 0.0;
	odom_pose.pose.orientation.w = 1.0;


	ros::Time now = ros::Time::now();
	tf_.waitForTransform("/fake_map", "/petit_base_link", now, ros::Duration(5.0));
	geometry_msgs::PoseStamped base_pose;
	tf_.transformPose("/fake_map", odom_pose, base_pose);

	if(color_name == "Yellow") {
		/* Initialize the starting pose */
		ROS_ERROR("YELLOW x : %f / y : %f", base_pose.pose.position.x, base_pose.pose.position.y);

		t = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(YELLOW_START_Z - 2*acos(base_pose.pose.orientation.w)),  
					tf::Vector3(YELLOW_START_X - (base_pose.pose.position.x*cos(YELLOW_START_Z - 2*acos(base_pose.pose.orientation.w) ) 
					- base_pose.pose.position.y*sin(YELLOW_START_Z - 2*acos(base_pose.pose.orientation.w) )), 
					YELLOW_START_Y - (base_pose.pose.position.y*cos(YELLOW_START_Z - 2*acos(base_pose.pose.orientation.w)) 
					+ base_pose.pose.position.x*sin(YELLOW_START_Z - 2*acos(base_pose.pose.orientation.w) ) ), 0.0)), 
				ros::Time::now(), "/petit_map", "/fake_map");

		broadcaster.sendTransform(t);
		init_done = 1;
	}
	else {                
		/* Initialize the starting pose */
		ROS_ERROR("RED x : %f / y : %f", base_pose.pose.position.x, base_pose.pose.position.y);

                t = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(RED_START_Z - 2*acos(base_pose.pose.orientation.w)),
                                        tf::Vector3(RED_START_X - (base_pose.pose.position.x*cos(RED_START_Z - 2*acos(base_pose.pose.orientation.w) )
                                        - base_pose.pose.position.y*sin(RED_START_Z - 2*acos(base_pose.pose.orientation.w) )),
                                        RED_START_Y - (base_pose.pose.position.y*cos(RED_START_Z - 2*acos(base_pose.pose.orientation.w))
                                        + base_pose.pose.position.x*sin(RED_START_Z - 2*acos(base_pose.pose.orientation.w) ) ), 0.0)),
                                ros::Time::now(), "/petit_map", "/fake_map");

		broadcaster.sendTransform(t);
		init_done = 1;
	}

	usleep(10000);

	std_msgs::Empty empty_mess;
	start_path_pub.publish(empty_mess);

}

void InitPose::broadcast(void)
{
	if( init_done == 1) {
		t.stamp_ = ros::Time::now();
		broadcaster.sendTransform(t);
	}
}

int main(int argc, char **argv)
{


	ros::init(argc, argv, "PETIT_InitPose");

	tf::TransformListener listener(ros::Duration(10));
	InitPose initpose(listener);
	// Refresh rate
	ros::Rate loop_rate(20);                                // 35 with bluetooth


	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		initpose.broadcast();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

