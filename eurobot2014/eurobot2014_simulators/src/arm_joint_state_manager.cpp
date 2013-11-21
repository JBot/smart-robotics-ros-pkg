#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sensor_msgs/JointState.h>


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
//#include <math.h>
//#include <algorithm>

#include <vector>

class simulatorARM {
	public:
		simulatorARM();

		ros::Subscriber Rjoint1_sub_;
		ros::Subscriber Rjoint2_sub_;
		ros::Subscriber Rjoint3_sub_;
		ros::Subscriber Rjoint4_sub_;
		ros::Subscriber Rjoint5_sub_;

		ros::Subscriber Ljoint1_sub_;
		ros::Subscriber Ljoint2_sub_;
		ros::Subscriber Ljoint3_sub_;
		ros::Subscriber Ljoint4_sub_;
		ros::Subscriber Ljoint5_sub_;

		ros::Publisher joint_pub;

		void joint_publish(void);


	private:
		void Rjoint1Callback(const std_msgs::Float64::ConstPtr & pose);
		void Rjoint2Callback(const std_msgs::Float64::ConstPtr & pose);
		void Rjoint3Callback(const std_msgs::Float64::ConstPtr & pose);
		void Rjoint4Callback(const std_msgs::Float64::ConstPtr & pose);
		void Rjoint5Callback(const std_msgs::Float64::ConstPtr & pose);

		void Ljoint1Callback(const std_msgs::Float64::ConstPtr & pose);
		void Ljoint2Callback(const std_msgs::Float64::ConstPtr & pose);
		void Ljoint3Callback(const std_msgs::Float64::ConstPtr & pose);
		void Ljoint4Callback(const std_msgs::Float64::ConstPtr & pose);
		void Ljoint5Callback(const std_msgs::Float64::ConstPtr & pose);

		ros::NodeHandle nh;
		sensor_msgs::JointState joint_state;


};

simulatorARM::simulatorARM()
{


	Rjoint1_sub_ = nh.subscribe < std_msgs::Float64 > ("/Rlink1_controller/command", 5, &simulatorARM::Rjoint1Callback, this);
	Rjoint2_sub_ = nh.subscribe < std_msgs::Float64 > ("/Rlink2_controller/command", 5, &simulatorARM::Rjoint2Callback, this);
	Rjoint3_sub_ = nh.subscribe < std_msgs::Float64 > ("/Rlink3_controller/command", 5, &simulatorARM::Rjoint3Callback, this);
	Rjoint4_sub_ = nh.subscribe < std_msgs::Float64 > ("/Rlink4_controller/command", 5, &simulatorARM::Rjoint4Callback, this);
	Rjoint5_sub_ = nh.subscribe < std_msgs::Float64 > ("/Rlink5_controller/command", 5, &simulatorARM::Rjoint5Callback, this);

	Ljoint1_sub_ = nh.subscribe < std_msgs::Float64 > ("/Llink1_controller/command", 5, &simulatorARM::Ljoint1Callback, this);
	Ljoint2_sub_ = nh.subscribe < std_msgs::Float64 > ("/Llink2_controller/command", 5, &simulatorARM::Ljoint2Callback, this);
	Ljoint3_sub_ = nh.subscribe < std_msgs::Float64 > ("/Llink3_controller/command", 5, &simulatorARM::Ljoint3Callback, this);
	Ljoint4_sub_ = nh.subscribe < std_msgs::Float64 > ("/Llink4_controller/command", 5, &simulatorARM::Ljoint4Callback, this);
	Ljoint5_sub_ = nh.subscribe < std_msgs::Float64 > ("/Llink5_controller/command", 5, &simulatorARM::Ljoint5Callback, this);

	joint_pub = nh.advertise < sensor_msgs::JointState > ("joint_states", 1);

	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(11);
	joint_state.position.resize(11);
	joint_state.velocity.resize(11);
	joint_state.effort.resize(11);

	joint_state.name[0] ="Rarm1_joint";
	joint_state.name[1] ="Rarm2_joint";
	joint_state.name[2] ="Rarm3_joint";
	joint_state.name[3] ="Rarm4_joint";
	joint_state.name[4] ="Rarm5_joint";

	joint_state.name[5] ="Larm1_joint";
	joint_state.name[6] ="Larm2_joint";
	joint_state.name[7] ="Larm3_joint";
	joint_state.name[8] ="Larm4_joint";
	joint_state.name[9] ="Larm5_joint";

	joint_state.name[10] ="base_laser_joint";

	joint_state.position[0] = 0.0;
	joint_state.position[1] = 0.0;
	joint_state.position[2] = 0.0;
	joint_state.position[3] = 0.0;
	joint_state.position[4] = 0.0;

	joint_state.position[5] = 0.0;
	joint_state.position[6] = 0.0;
	joint_state.position[7] = 0.0;
	joint_state.position[8] = 0.0;
	joint_state.position[9] = 0.0;

	joint_state.position[10] = 0.0;

	joint_pub.publish(joint_state);




}


void simulatorARM::Rjoint1Callback(const std_msgs::Float64::ConstPtr & pose)
{
	//ROS_ERROR("Value %f", (pose->motor_states[0].position*0.292968 - 150.0)*3.141592654/180);
	joint_state.position[0] = pose->data;
}

void simulatorARM::Rjoint2Callback(const std_msgs::Float64::ConstPtr & pose)
{
	joint_state.position[1] = pose->data;
}

void simulatorARM::Rjoint3Callback(const std_msgs::Float64::ConstPtr & pose)
{
	joint_state.position[2] = pose->data;
}

void simulatorARM::Rjoint4Callback(const std_msgs::Float64::ConstPtr & pose)
{
	joint_state.position[3] = pose->data;
}

void simulatorARM::Rjoint5Callback(const std_msgs::Float64::ConstPtr & pose)
{
	joint_state.position[4] = pose->data;
}

void simulatorARM::Ljoint1Callback(const std_msgs::Float64::ConstPtr & pose)
{
	joint_state.position[5] = pose->data;
}

void simulatorARM::Ljoint2Callback(const std_msgs::Float64::ConstPtr & pose)
{
	joint_state.position[6] = pose->data;
}

void simulatorARM::Ljoint3Callback(const std_msgs::Float64::ConstPtr & pose)
{
	joint_state.position[7] = pose->data;
}

void simulatorARM::Ljoint4Callback(const std_msgs::Float64::ConstPtr & pose)
{
	joint_state.position[8] = pose->data;
}

void simulatorARM::Ljoint5Callback(const std_msgs::Float64::ConstPtr & pose)
{
	joint_state.position[9] = pose->data;
}

void simulatorARM::joint_publish(void)
{

	joint_state.header.stamp = ros::Time::now();
	/*    joint_state.name.resize(4);
	      joint_state.position.resize(4);
	      joint_state.velocity.resize(4);
	      joint_state.effort.resize(4);
	 */
	/*
	//joint_state.position[0] = 0;
	joint_state.position[1] = 0;
	joint_state.position[2] = 0;
	joint_state.position[3] = 0;
	 */
	joint_pub.publish(joint_state);
}



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
	ros::init(argc, argv, "ARM_simulator");
	simulatorARM simulatorarm;
	// Refresh rate
	ros::Rate loop_rate(10);                                // 35 with bluetooth
	while (ros::ok()) {
		simulatorarm.joint_publish();
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}




