#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <tf/transform_broadcaster.h>

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


class PoseSimulator {
	public:
		PoseSimulator();
		void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose);
		void publish_all(void);

		// Set the position of the robot
		ros::Subscriber pose_in_map_sub_;
		ros::Subscriber pose_sub_;
		// Set the position of the robot (velocity)
		ros::Subscriber vel_sub_;


	private:
		void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
		void pose2DCallback(const geometry_msgs::Pose2D::ConstPtr & pose);
		void velCallback(const geometry_msgs::Twist::ConstPtr & pose);
		ros::NodeHandle nh;

		tf::TransformBroadcaster br;
		tf::Transform transform;
		geometry_msgs::PoseStamped temp_pose;
		geometry_msgs::Twist temp_twist;
		double temp_heading;
		
		std::string map_name;
		std::string base_name;

};

PoseSimulator::PoseSimulator()
{
	ros::NodeHandle nhp("~");

        nhp.param<std::string>("map_name", map_name, "/map");
        nhp.param<std::string>("base_name", base_name, "/base_link");
        //nhp.getParam("map_name", map_name);

	// Set the position of the robot
	pose_in_map_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("/ROBOT/startpose", 20, &PoseSimulator::poseCallback, this);
	pose_sub_ = nh.subscribe < geometry_msgs::Pose2D > ("/ROBOT/pose", 20, &PoseSimulator::pose2DCallback, this);
	// Set the position of the robot (velocity)
	vel_sub_ = nh.subscribe < geometry_msgs::Twist > ("/ROBOT/cmd_vel", 20, &PoseSimulator::velCallback, this);

	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	transform.setRotation(tf::Quaternion(0, 0, 0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_name, base_name));

	temp_pose.pose.position.x = 0;
	temp_pose.pose.position.y = 0;
	temp_pose.pose.position.z = 0;
	temp_pose.pose.orientation.x = 0;
	temp_pose.pose.orientation.y = 0;
	temp_pose.pose.orientation.z = 0;
	temp_pose.pose.orientation.w = 1;

	temp_heading = 0.0;


}


void PoseSimulator::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose)
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

void PoseSimulator::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose)
{
	temp_pose.pose.position.x = pose->pose.position.x;
	temp_pose.pose.position.y = pose->pose.position.y;
	temp_pose.pose.orientation.x = pose->pose.orientation.x;
	temp_pose.pose.orientation.y = pose->pose.orientation.y;
	temp_pose.pose.orientation.z = pose->pose.orientation.z;
	temp_pose.pose.orientation.w = pose->pose.orientation.w;

	// TO VERIFY
	double tmp1 = 2.0*pose->pose.orientation.z*pose->pose.orientation.w;
	double tmp2 = 2.0*pose->pose.orientation.x*pose->pose.orientation.z;
	double tmp3 = 2.0*pose->pose.orientation.y*pose->pose.orientation.y;
	double tmp4 = 2.0*pose->pose.orientation.z*pose->pose.orientation.z;

	temp_heading = atan2(tmp1 - tmp2, 1.0 - tmp3 - tmp4);
}

void PoseSimulator::pose2DCallback(const geometry_msgs::Pose2D::ConstPtr & pose)
{


        temp_pose.pose.position.x = pose->x;
        temp_pose.pose.position.y = pose->y;

	if(pose->theta > 9999) {
	}
	else {
		rotate(0.0, pose->theta, 0.0, &temp_pose);

        	temp_heading = pose->theta;
	}

	int i = 0;
	while(i < 10) {
		//ros::spinOnce();
		usleep(50000);
		i++;
	}
}


void PoseSimulator::velCallback(const geometry_msgs::Twist::ConstPtr & pose)
{
	temp_twist = *pose;
}


void PoseSimulator::publish_all(void)
{
	temp_pose.pose.position.x = temp_pose.pose.position.x + cos(temp_heading) * temp_twist.linear.x / 20 - sin(temp_heading) * temp_twist.linear.y / 20;
	temp_pose.pose.position.y = temp_pose.pose.position.y + cos(temp_heading) * temp_twist.linear.y / 20 + sin(temp_heading) * temp_twist.linear.x / 20;

	temp_heading = temp_heading + temp_twist.angular.z / 20;

	rotate(0.0, temp_heading, 0.0, &temp_pose);

	transform.setOrigin(tf::Vector3(temp_pose.pose.position.x, temp_pose.pose.position.y, 0.0));
	transform.setRotation(tf::Quaternion(temp_pose.pose.orientation.x, temp_pose.pose.orientation.y,
				temp_pose.pose.orientation.z, temp_pose.pose.orientation.w));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), map_name, base_name));
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
	ros::init(argc, argv, "Pose_Simulator");
	PoseSimulator posesimulator;
	// Refresh rate
	ros::Rate loop_rate(20);                                // 35 with bluetooth
	float rotation = 0.0;
	while (ros::ok()) {
		posesimulator.publish_all();

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}
