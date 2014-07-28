#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Quaternion.h>
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


class InitPose {
	public:
		InitPose(tf::TransformListener& tf);

		ros::Subscriber quat_sub;

		void broadcast(void);

	private:
		void quatCallback(const geometry_msgs::Quaternion::ConstPtr& quat);

		ros::NodeHandle nh;

		tf::TransformListener& tf_;
		tf::StampedTransform t;
		tf::TransformBroadcaster broadcaster;

		std::string color_name;

		int8_t init_done;

		float x, y, z, tx, ty, tz;

};

InitPose::InitPose(tf::TransformListener& tf) :
	tf_(tf)
{
	quat_sub = nh.subscribe<geometry_msgs::Quaternion>("/CALIB/PlanModel", 5, &InitPose::quatCallback, this);

	// First guess of the kinect position
	x = 0.00;
	y = 0.00;
	z = 0.50;
	tx = 0.00;
	ty = 0.60;
	tz = 0.00;

	// Create the transform
	t = tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(tx, ty, tz), tf::Vector3( x, y, z)), ros::Time::now(), "/map", "/camera_link");

}


void InitPose::quatCallback(const geometry_msgs::Quaternion::ConstPtr& quat)
{

	// Adjust the x angle depending on the b value of plane model
	if(fabs(quat->y) > 0.01) {
		tx = tx + quat->y/2;
	}

	// Adjust the y angle depending on the a value of plane model
	if(fabs(quat->x) > 0.01) {
		ty = ty - quat->x/2;
	}

	/*
	   if(fabs(quat->z) > 0.03) {
	   tz = tz + quat->z;
	   }
	 */

	// Adjust the z position depending on the d value of plane model
	if(fabs(quat->w) > 0.005) {
		z = z + quat->w/2;
	}

	// Update transform
	t = tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(tx, ty, tz), tf::Vector3( x, y, z)), ros::Time::now(), "/map", "/camera_link");
	broadcaster.sendTransform(t);


}

void InitPose::broadcast(void)
{
	// Broadcast transform with the current time
	t.stamp_ = ros::Time::now();
	broadcaster.sendTransform(t);
}

int main(int argc, char **argv)
{


	ros::init(argc, argv, "kinect_InitPose");

	tf::TransformListener listener(ros::Duration(10));
	InitPose initpose(listener);
	// Refresh rate
	ros::Rate loop_rate(50);


	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		initpose.broadcast();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

