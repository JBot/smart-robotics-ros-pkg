#include "ros/ros.h"
#include <std_msgs/UInt16.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cmath>

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



class StandardServo {
	public:
		StandardServo();
		~StandardServo();

		ros::Subscriber goal_sub_;
		ros::Publisher goal_pub;
	private:
		void goalCallback(const std_msgs::UInt16::ConstPtr & goal);

		ros::NodeHandle nh;
		int inverted; // 0 = no / 1 = yes
		int offset;

};

StandardServo::StandardServo()
{
	ros::NodeHandle nhp("~");

	nhp.param<int>("inverted", inverted, 0);
	nhp.param<int>("offset", offset, 0);

	goal_sub_ = nh.subscribe < std_msgs::UInt16 > ("/servo_in", 2, &StandardServo::goalCallback, this);
	goal_pub = nh.advertise < std_msgs::UInt16 > ("/servo_out", 2);


}

StandardServo::~StandardServo()
{
}


void StandardServo::goalCallback(const std_msgs::UInt16::ConstPtr & goal)
{
	std_msgs::UInt16 out;
	out.data = goal->data;
	if(inverted) {
		out.data = 180 - out.data;
	}
	out.data = out.data + offset;
	goal_pub.publish(out);
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
	ros::init(argc, argv, "StandardServo");
	StandardServo standardservo;

	ros::spin();

	ros::shutdown();
}



