/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

/* C includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

/* C++ includes */
#include <iostream>
#include <string>
#include <list>

class nestorCalendar {
	public:
		nestorCalendar();

		ros::Publisher english_pub;
		ros::Publisher french_pub;
		ros::Publisher command_pub;

		void check(void);
	private:

		ros::NodeHandle nh;
};


/* Constructor */
nestorCalendar::nestorCalendar()
{
	english_pub = nh.advertise < std_msgs::String > ("/nestor/english_voice", 3);
	french_pub = nh.advertise < std_msgs::String > ("/nestor/french_voice", 3);
	command_pub = nh.advertise < std_msgs::Int32 > ("/nestor/command", 3);
}


void nestorCalendar::check(void)
{

	FILE *fp;
	int status;
	char path[1035];
	std_msgs::String to_send;
	std_msgs::Int32 command_tosend;

	time_t rawtime;
	struct tm *info;
	char buffer[80];

	time( &rawtime );

	info = localtime( &rawtime );

	int start_h = info->tm_hour;
	int start_m = info->tm_min;
	int stop_h = info->tm_hour;
	int stop_m = info->tm_min + 5;
	if(stop_m > 59) {
		stop_m-=60;
		stop_h+=1;
	}
	if(stop_h > 23) {
		stop_h-=24;
	}

	sprintf (buffer, "gcalcli agenda %d:%d %d:%d", start_h, start_m, stop_h, stop_m);

	/* Open the command for reading. */
	fp = popen(buffer, "r");
	if (fp == NULL) {
		printf("Failed to run command\n" );
		exit;
	}

	int i = 0;	
	/* Read the output a line at a time - output it. */
	while (fgets(path, sizeof(path)-1, fp) != NULL) {
		//std::cout << path << std::endl;
		if( i == 1) {
			std::string my_string = path;
			if( my_string.substr(0, 15) == "No Events Found") {
				std::cout << "No new event !" << std::endl;
			}
			else {
				std::cout << "" << my_string << std::endl;
				int pos;
				std::string str3;
				if( my_string.find("command") != -1 ) {
					pos = my_string.find("command");
					str3 = my_string.substr(pos+7);

					std::cout << "" << str3 << std::endl;
					//if( str3.substr(0, 7) == "command" ) {
					std::cout << "new command : " << str3.substr(1, 1) << std::endl;
					command_tosend.data = atoi( str3.substr(1, 1).c_str() );
					switch(atoi( str3.substr(1, 1).c_str() )) {
						case 0 :
							std::cout << "0" << std::endl;
							command_pub.publish(command_tosend);
							break;
						case 1 :
							std::cout << "1" << std::endl;
							command_pub.publish(command_tosend);
							break;
						case 2 :
							std::cout << "2" << std::endl;
							command_pub.publish(command_tosend);
							break;
						default:
							std::cout << "default" << std::endl;
							command_pub.publish(command_tosend);
							break;
					}



				}
				else {
					str3 = my_string.substr(30);
					std::cout << "" << str3 << std::endl;
					// say something
					to_send.data = str3;
					french_pub.publish(to_send);
				}

			}
			//std::cout << str3 << std::endl;
		}
		i++;
	}

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
	ros::init(argc, argv, "Nestor_calendar");
	nestorCalendar nestor_calendar;
	// Refresh rate
	ros::Rate loop_rate(0.00333); /*  */
	while (ros::ok()) {
		nestor_calendar.check();
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

