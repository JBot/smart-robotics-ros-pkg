#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <maximus_position/AvrPose.h>


#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                               // for in-/output
#include <string.h>                              // strcat

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>

#include <vector>


class TransformPose
{
	public:
		TransformPose();
		void publish_all(void);

		// Joystick suscriber
		ros::Subscriber avrpose_sub_;
		// Set a Laser scan sensor for the robot
		ros::Publisher pose_pub;


	private:
		void poseCallback(const maximus_position::AvrPose::ConstPtr& pose);
		void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped *pose);
		ros::NodeHandle nh;

		geometry_msgs::PoseStamped my_maximus_pose;
};

TransformPose::TransformPose()
{

	// Joystick suscriber
	avrpose_sub_ = nh.subscribe<maximus_position::AvrPose>("avrpose", 10, &TransformPose::poseCallback, this);
	// Set a Laser scan sensor for the robot
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("avr_maximus_pose", 20);

	my_maximus_pose.header.frame_id = "/map";
	my_maximus_pose.header.stamp = ros::Time::now();

	my_maximus_pose.pose.position.x = 0;
        my_maximus_pose.pose.position.y = 0;
        my_maximus_pose.pose.position.z = 0;
        my_maximus_pose.pose.orientation.x = 0;
        my_maximus_pose.pose.orientation.y = 0;
        my_maximus_pose.pose.orientation.z = 0;

}

void TransformPose::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped *pose) {
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


void TransformPose::poseCallback(const maximus_position::AvrPose::ConstPtr& pose)
{
	float tmp = 0.0;

	my_maximus_pose.pose.position.x = pose->x/1000;
        my_maximus_pose.pose.position.y = pose->y/1000;
        my_maximus_pose.pose.position.z = 0;

	TransformPose::rotate(0, (pose->theta), 0, &my_maximus_pose);

	my_maximus_pose.header.stamp = ros::Time::now();
	pose_pub.publish(my_maximus_pose);

}


void TransformPose::publish_all(void) {

	// Publish the laser scan values
	TransformPose::pose_pub.publish(my_maximus_pose);
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
	ros::init(argc, argv, "transformPose");
	TransformPose transform_Pose;
	// Refresh rate
	ros::Rate loop_rate(5); // 35 with bluetooth
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}




