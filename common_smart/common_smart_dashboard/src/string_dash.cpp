#include "ros/ros.h"
#include <std_msgs/String.h>

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



class Dash {
	public:
		Dash();

		ros::Subscriber string_sub_;

	private:
		void stringCallback(const std_msgs::String::ConstPtr &value);
		ros::NodeHandle nh;

		std::string address, port, type, dash_topic;
};

Dash::Dash()
{

	ros::NodeHandle nhp("~");
	//nhp.getParam("base_name", base_name);

	nhp.param<std::string>("address", address, "192.168.0.1");
	nhp.param<std::string>("port", port, "3030");
	nhp.param<std::string>("type", type, "text");
	nhp.param<std::string>("dash_topic", dash_topic, "welcome");

	string_sub_ = nh.subscribe < std_msgs::String > ("/DASHBOARD/stringtest", 1, &Dash::stringCallback, this);
}

void Dash::stringCallback(const std_msgs::String::ConstPtr &value)
{
	char my_buff[256];
	sprintf(my_buff, "curl -d \'{ \"auth_token\": \"YOUR_AUTH_TOKEN\", \"%s\": \"%s\" }' \\http://%s:%s/widgets/%s", 
			type.c_str(),
			value->data.c_str(),
			address.c_str(),
			port.c_str(),
			dash_topic.c_str());
	std::cout << my_buff << std::endl;
	system(my_buff);
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
	ros::init(argc, argv, "String_Dash");

	Dash dash;

	// Refresh rate
	ros::Rate loop_rate(0.5);
	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}
