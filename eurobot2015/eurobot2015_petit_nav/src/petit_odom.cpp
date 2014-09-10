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
		ros::Publisher odom_pub;

		void broadcast(void);

	private:
		void imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr& imu);
		void velCallback(const geometry_msgs::Twist::ConstPtr& vel);

		ros::NodeHandle nh;

		nav_msgs::Odometry my_odom;

		tf::StampedTransform t;
		tf::TransformBroadcaster broadcaster;

		ros::Time current_time, last_time;

		double x, y, z;
		double vx, vy, vz;
		double th, yaw;
		double vth;

		bool use_imu;
};

OdomPose::OdomPose()
{
	//ros::NodeHandle nhp("~");

	ros::NodeHandle nhp("~");

	nhp.param<bool>("use_imu", use_imu, false);


	imu_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("imu/rpy", 5, &OdomPose::imuCallback, this);
	cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/PETIT/cmd_vel", 5, &OdomPose::velCallback, this);

	odom_pub = nh.advertise < nav_msgs::Odometry > ("/PETIT/odom", 50);

	my_odom.header.frame_id = "/petit_odom_link";
	my_odom.child_frame_id = "/petit_base_link";
	my_odom.header.stamp = ros::Time::now();

	my_odom.pose.pose.position.x = 0;
	my_odom.pose.pose.position.y = 0;
	my_odom.pose.pose.position.z = 0;
	my_odom.pose.pose.orientation.x = 0;
	my_odom.pose.pose.orientation.y = 0;
	my_odom.pose.pose.orientation.z = 0;
	my_odom.pose.pose.orientation.w = 0;

	my_odom.twist.twist.linear.x = 0;
	my_odom.twist.twist.linear.y = 0;
	my_odom.twist.twist.linear.z = 0;
	my_odom.twist.twist.angular.x = 0;
	my_odom.twist.twist.angular.y = 0;
	my_odom.twist.twist.angular.z = 0;

	usleep(5);
	last_time = ros::Time::now();
	x = y = z = 0.0;
	vx = vy = vz = 0.0;
	th = 0.0;
	vth = 0.0;
	yaw = 0.0;



}


void OdomPose::imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr& imu)
{

	if( use_imu ) {
		yaw = imu->vector.z;
	}
	/*t = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(yaw), tf::Vector3(x, y, z)),
	  ros::Time::now(), "/petit_odom_link", "/petit_base_link");

	  t.stamp_ = ros::Time::now();
	  broadcaster.sendTransform(t);
	 */
}

void OdomPose::velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
	current_time = ros::Time::now();
	//	double dt = (current_time - last_time).toSec();

	vx = vel->linear.x;
	vy = vel->linear.y;
	vth = vel->angular.z;
	/*
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
	 */
}

void OdomPose::broadcast(void)
{
	current_time = ros::Time::now();

	if( (current_time - last_time).toSec() > 0.02) {

		double dt = (current_time - last_time).toSec();

		double delta_x = (vx * cos(yaw) - vy * sin(yaw)) * dt;
		double delta_y = (vx * sin(yaw) + vy * cos(yaw)) * dt;
		double delta_th = vth * dt;

		if( use_imu ) {
			vth = 0.0;
			vth = (yaw - th) / dt;
			th = yaw;
		} 
		else {
			th += delta_th;
			yaw = th;
		}
		x += delta_x;
		y += delta_y;


		my_odom.header.stamp = current_time;
                my_odom.pose.pose.position.x = x;
                my_odom.pose.pose.position.y = y;
                my_odom.pose.pose.position.z = 0.0;
                my_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

                my_odom.twist.twist.linear.x = vx;
                my_odom.twist.twist.linear.y = vy;
                my_odom.twist.twist.linear.z = 0.0;
                my_odom.twist.twist.angular.x = 0.0;
                my_odom.twist.twist.angular.y = 0.0;
                my_odom.twist.twist.angular.z = vth;

                odom_pub.publish(my_odom);


		t = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(yaw), tf::Vector3(x, y, z)),
				ros::Time::now(), "/petit_odom_link", "/petit_base_link");

		t.stamp_ = ros::Time::now();
		broadcaster.sendTransform(t);

		last_time = current_time;
	}
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

		odompose.broadcast();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

