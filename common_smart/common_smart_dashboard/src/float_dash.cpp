#include "ros/ros.h"
#include <std_msgs/Float32.h>

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

		ros::Subscriber float_sub_;

	private:
		void floatCallback(const std_msgs::Float32::ConstPtr &value);
		ros::NodeHandle nh;

		double value_save;

		std::string address, port, type, dash_topic;

};

Dash::Dash()
{
	ros::NodeHandle nhp("~");
	//nhp.getParam("base_name", base_name);

	nhp.param<std::string>("address", address, "192.168.0.1");
	nhp.param<std::string>("port", port, "3030");
	nhp.param<std::string>("type", type, "value");
	nhp.param<std::string>("dash_topic", dash_topic, "synergie");

	float_sub_ = nh.subscribe < std_msgs::Float32 > ("/DASHBOARD/test", 1, &Dash::floatCallback, this);

	value_save = 0.0;
}

void Dash::floatCallback(const std_msgs::Float32::ConstPtr &value)
{
	char my_buff[256];

	if(fabs(value->data - value_save) > 0.01)
	{
		sprintf(my_buff, "curl -d \'{ \"auth_token\": \"YOUR_AUTH_TOKEN\", \"%s\": \"%f\" }' \\http://%s:%s/widgets/%s", 
				type.c_str(), 
				value->data, 
				address.c_str(), 
				port.c_str(), 
				dash_topic.c_str());

		value_save = value->data;


		std::cout << my_buff << std::endl;
		system(my_buff);

	}

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
	ros::init(argc, argv, "Dash");

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
