#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

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


class OdomPose {
	public:
		OdomPose();

		ros::Subscriber imu_sub;
		ros::Subscriber cmd_vel_sub;

		void broadcast(void);

	private:
		void imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr& imu);
		void velCallback(const geometry_msgs::Twist::ConstPtr& vel);

		ros::NodeHandle nh;

		tf::StampedTransform t;
		tf::TransformBroadcaster broadcaster;

		ros::Time current_time, last_time;

		double x, y, z;
		double th, yaw;

};

OdomPose::OdomPose()
{
	//ros::NodeHandle nhp("~");

	imu_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("imu/rpy", 5, &OdomPose::imuCallback, this);
	cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/PETIT/cmd_vel", 5, &OdomPose::velCallback, this);

	usleep(5);
	last_time = ros::Time::now();
	x = y = z = 0.0;
	th = 0.0;
	yaw = 0.0;

}


void OdomPose::imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr& imu)
{
		yaw = imu->vector.z;
                t = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(yaw), tf::Vector3(x, y, z)),
                                ros::Time::now(), "/petit_odom_link", "/petit_base_link");

		t.stamp_ = ros::Time::now();
                broadcaster.sendTransform(t);

}

void OdomPose::velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();

	double delta_x = (vel->linear.x * cos(yaw) - vel->linear.y * sin(yaw)) * dt;
    	double delta_y = (vel->linear.x * sin(yaw) + vel->linear.y * cos(yaw)) * dt;
    	double delta_th = vel->angular.z * dt;

	x += delta_x;
    	y += delta_y;
    	th += delta_th;

	t = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(yaw), tf::Vector3(x, y, z)),
                                ros::Time::now(), "/petit_odom_link", "/petit_base_link");

        t.stamp_ = ros::Time::now();
        broadcaster.sendTransform(t);

	last_time = current_time;
}


int main(int argc, char **argv)
{


	ros::init(argc, argv, "PETIT_OdomPose");

	OdomPose odompose;
	// Refresh rate
	ros::Rate loop_rate(110);                                // 35 with bluetooth


	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

