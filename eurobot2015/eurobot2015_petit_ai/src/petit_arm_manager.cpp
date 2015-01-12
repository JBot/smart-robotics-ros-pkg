#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int32.h"


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
#include <map>

#define FRONT	0
#define REAR	1
#define RIGHT	2
#define LEFT	3


using namespace std;


class ARMManager {
	public:
		ARMManager();
		void loop(void);

		ros::Publisher enable_servo;
		ros::Publisher servo_speed;

		ros::Publisher servo1;
		ros::Publisher servo2;
		ros::Publisher servo3;
		ros::Publisher servo4;
		ros::Publisher servo5;
		ros::Publisher servo6;
		ros::Publisher servo7;
		ros::Publisher servo8;


		ros::Subscriber setside_sub;
		ros::Subscriber start_sub;
		ros::Subscriber grab_sub;
		ros::Subscriber release_sub;
		ros::Subscriber collect_sub;
		ros::Subscriber climb_sub;
		ros::Subscriber color_sub;


	private:
		void setsideCallback(const std_msgs::Int32::ConstPtr &my_int);
		void startCallback(const std_msgs::Empty::ConstPtr &value);
		void grabCallback(const std_msgs::Int32::ConstPtr &my_int);
		void releaseCallback(const std_msgs::Int32::ConstPtr &my_int);
		void collectCallback(const std_msgs::Empty::ConstPtr &value);
		void climbCallback(const std_msgs::Empty::ConstPtr &value);
		void colorCallback(const std_msgs::Int32::ConstPtr &my_int);

		int32_t side;

		ros::NodeHandle nh;


};

ARMManager::ARMManager()
{
	ros::NodeHandle nhp("~");


	enable_servo = nh.advertise < std_msgs::Empty > ("/ROBOT/ARM/enable_servo", 10);
	servo_speed = nh.advertise < std_msgs::UInt16 > ("/ROBOT/ARM/servo_speed", 10);

	servo1 = nh.advertise < std_msgs::UInt16 > ("/ROBOT/ARM/servo1", 10);
	servo2 = nh.advertise < std_msgs::UInt16 > ("/ROBOT/ARM/servo2", 10);
	servo3 = nh.advertise < std_msgs::UInt16 > ("/ROBOT/ARM/servo3", 10);
	servo4 = nh.advertise < std_msgs::UInt16 > ("/ROBOT/ARM/servo4", 10);
	servo5 = nh.advertise < std_msgs::UInt16 > ("/ROBOT/ARM/servo5", 10);
	servo6 = nh.advertise < std_msgs::UInt16 > ("/ROBOT/ARM/servo6", 10);
	servo7 = nh.advertise < std_msgs::UInt16 > ("/ROBOT/ARM/servo7", 10);
	servo8 = nh.advertise < std_msgs::UInt16 > ("/ROBOT/ARM/servo8", 10);

	setside_sub = nh.subscribe < std_msgs::Int32 > ("/ROBOT/ARM/set_side", 20, &ARMManager::setsideCallback, this);
	start_sub = nh.subscribe < std_msgs::Empty > ("/GENERAL/start", 20, &ARMManager::startCallback, this);
	grab_sub = nh.subscribe < std_msgs::Int32 > ("/ROBOT/ARM/grab", 20, &ARMManager::grabCallback, this);
	release_sub = nh.subscribe < std_msgs::Int32 > ("/ROBOT/ARM/release", 20, &ARMManager::releaseCallback, this);
	collect_sub = nh.subscribe < std_msgs::Empty > ("/ROBOT/ARM/collect", 20, &ARMManager::collectCallback, this);
	climb_sub = nh.subscribe < std_msgs::Empty > ("/ROBOT/ARM/climb", 20, &ARMManager::climbCallback, this);
	color_sub = nh.subscribe < std_msgs::Int32 > ("/GENERAL/color", 20, &ARMManager::colorCallback, this);

	side = FRONT;

}


void ARMManager::setsideCallback(const std_msgs::Int32::ConstPtr & my_int)
{
	std_msgs::UInt16 output;
	switch(my_int->data) 
	{
		case FRONT:
			switch(side)
			{
				case FRONT:
					break;
				case REAR:
					// TODO
					output.data = 1;
					servo3.publish(output);
					output.data = 95;
					servo4.publish(output);
					output.data = 1;
					servo5.publish(output);
					output.data = 5;
					servo6.publish(output);
					output.data = 95;
					servo7.publish(output);
					output.data = 20;
					servo8.publish(output);
					break;
				case RIGHT:
					break;
				case LEFT:
					break;
				default:
					break;
			}
			output.data = 1;
			servo3.publish(output);
			output.data = 95;
			servo4.publish(output);
			output.data = 1;
			servo5.publish(output);
			output.data = 5;
			servo6.publish(output);
			output.data = 95;
			servo7.publish(output);
			output.data = 20;
			servo8.publish(output);
			side = FRONT;
			break;
		case REAR:
			switch(side)
			{
				case FRONT:
					output.data = 1;
					servo3.publish(output);
					output.data = 150;
					servo4.publish(output);
					output.data = 150;
					servo5.publish(output);

					usleep(500000);

					output.data = 5;
					servo6.publish(output);
					output.data = 150;
					servo7.publish(output);
					output.data = 150;
					servo8.publish(output);

					usleep(500000);

					break;
				case REAR:
					break;
				case RIGHT:
					break;
				case LEFT:
					break;
				default:
					break;
			}
			output.data = 170;
			servo3.publish(output);
			output.data = 97;
			servo4.publish(output);
			output.data = 1;
			servo5.publish(output);
			output.data = 170;
			servo6.publish(output);
			output.data = 100;
			servo7.publish(output);
			output.data = 150;
			servo8.publish(output);
			side = REAR;
			break;
		case RIGHT:
			switch(side)
			{
				case FRONT:
					break;
				case REAR:
					break;
				case RIGHT:
					break;
				case LEFT:
					break;
				default:
					break;
			}
			side = RIGHT;
			break;
		case LEFT:
			switch(side)
			{
				case FRONT:
					break;
				case REAR:
					break;
				case RIGHT:
					break;
				case LEFT:
					break;
				default:
					break;
			}
			side = LEFT;
			break;
		default:
			break;
	}

}

void ARMManager::colorCallback(const std_msgs::Int32::ConstPtr & my_int)
{
	// OK
	std_msgs::UInt16 output;

	output.data = 90;
	servo1.publish(output);
	output.data = 90;
	servo2.publish(output);
	output.data = 1;
	servo3.publish(output);
	output.data = 150;
	servo4.publish(output);
	output.data = 1;
	servo5.publish(output);
	output.data = 5;
	servo6.publish(output);
	output.data = 150;
	servo7.publish(output);
	output.data = 20;
	servo8.publish(output);
}

void ARMManager::grabCallback(const std_msgs::Int32::ConstPtr & my_int)
{
	// 
	std_msgs::UInt16 output;
	if(my_int->data == 1) // Front
	{
		output.data = 60; // TODO
		servo2.publish(output);
	}
	else {
		output.data = 60;
		servo1.publish(output);
	}

}

void ARMManager::releaseCallback(const std_msgs::Int32::ConstPtr & my_int)
{
	// OK
	std_msgs::UInt16 output;
	if(my_int->data == 1) // Front
	{
		output.data = 90;
		servo2.publish(output);
	}
	else {
		output.data = 90;
		servo1.publish(output);
	}

}

void ARMManager::startCallback(const std_msgs::Empty::ConstPtr &value)
{
	// OK
	std_msgs::UInt16 output;
	std_msgs::Empty out_enable;

	enable_servo.publish(out_enable);
	usleep(10000);

	output.data = 90;
	servo1.publish(output);
	output.data = 90;
	servo2.publish(output);
	output.data = 1;
	servo3.publish(output);
	output.data = 95;
	servo4.publish(output);
	output.data = 1;
	servo5.publish(output);
	output.data = 5;
	servo6.publish(output);
	output.data = 95;
	servo7.publish(output);
	output.data = 20;
	servo8.publish(output);
}

void ARMManager::collectCallback(const std_msgs::Empty::ConstPtr &value)
{
	if(side == REAR)
	{

	}
	else
	{

	}
}

void ARMManager::climbCallback(const std_msgs::Empty::ConstPtr &value)
{
	std_msgs::UInt16 output;
	if(side == REAR)
	{
		// TODO
		output.data = 90;
		servo1.publish(output);
		output.data = 60;
		servo2.publish(output);
		output.data = 1;
		servo3.publish(output);
		output.data = 95;
		servo4.publish(output);
		output.data = 1;
		servo5.publish(output);
		output.data = 5;
		servo6.publish(output);
		output.data = 95;
		servo7.publish(output);
		output.data = 20;
		servo8.publish(output);

		usleep(500000);

	}
	output.data = 90;
	servo1.publish(output);
	output.data = 60;
	servo2.publish(output);
	output.data = 1;
	servo3.publish(output);
	output.data = 95;
	servo4.publish(output);
	output.data = 90;
	servo5.publish(output);
	output.data = 5;
	servo6.publish(output);
	output.data = 95;
	servo7.publish(output);
	output.data = 90;
	servo8.publish(output);

}



void ARMManager::loop(void) 
{

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
	ros::init(argc, argv, "PETIT_ARM_manager");

	ARMManager armmanager;

	// Refresh rate
	ros::Rate loop_rate(2);
	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
		armmanager.loop();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}



