#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include "dynamixel_controllers/SetSpeed.h"
#include "dynamixel_controllers/SetComplianceSlope.h"
#include "dynamixel_msgs/JointState.h"
#include "dynamixel_msgs/MotorStateList.h"

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

class indomptableARM {
	public:
		indomptableARM();

		ros::Subscriber joint_sub_;
		ros::Publisher joint_pub;

		void joint_publish(void);


	private:
		void jointCallback(const dynamixel_msgs::MotorStateList::ConstPtr & pose);

		ros::NodeHandle nh;
		sensor_msgs::JointState joint_state;


};

indomptableARM::indomptableARM()
{


	joint_sub_ = nh.subscribe < dynamixel_msgs::MotorStateList > ("/motor_states/arm_port", 5, &indomptableARM::jointCallback, this);
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


void indomptableARM::jointCallback(const dynamixel_msgs::MotorStateList::ConstPtr & pose)
{

	//ROS_ERROR("Value %f", (pose->motor_states[0].position*0.292968 - 150.0)*3.141592654/180);

	joint_state.position[0] = (pose->motor_states[0].position*0.292968 - 150)*3.141592654/180;
	joint_state.position[1] = (pose->motor_states[1].position*0.292968 - 150)*3.141592654/180;
	joint_state.position[2] = (pose->motor_states[11].position*0.292968 - 150)*3.141592654/180;
	joint_state.position[3] = (pose->motor_states[2].position*0.292968 - 150)*3.141592654/180;
	joint_state.position[4] = (pose->motor_states[3].position*0.292968 - 150)*3.141592654/180;

        joint_state.position[5] = (pose->motor_states[4].position*0.292968 - 150)*3.141592654/180;
        joint_state.position[6] = (pose->motor_states[5].position*0.292968 - 150)*3.141592654/180;
        joint_state.position[7] = (pose->motor_states[6].position*0.292968 - 150)*3.141592654/180;
        joint_state.position[8] = (pose->motor_states[7].position*0.292968 - 150)*3.141592654/180;
        joint_state.position[9] = (pose->motor_states[8].position*0.292968 - 150)*3.141592654/180;

	//ROS_ERROR("pose : %d", pose->motor_states[10].position);
	if(pose->motor_states[10].position < 315)
        	joint_state.position[10] = 0.0;
	if( (pose->motor_states[10].position > 315) && (pose->motor_states[10].position < 400) )
        	joint_state.position[10] = 0.03;
	if(pose->motor_states[10].position > 400)
        	joint_state.position[10] = 0.13;
}


void indomptableARM::joint_publish(void)
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
	ros::init(argc, argv, "joint_arm_publisher");
	indomptableARM indomptablearm;
	// Refresh rate
	ros::Rate loop_rate(50);                                // 35 with bluetooth
	while (ros::ok()) {
		indomptablearm.joint_publish();
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}




