#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>


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


class DriveLauncher {
	public:
		DriveLauncher();

	private:
		void powerOnCallback(const std_msgs::Empty::ConstPtr& empty);
		void powerOffCallback(const std_msgs::Empty::ConstPtr& empty);
		void fireCallback(const std_msgs::Empty::ConstPtr& empty);
		void openCallback(const std_msgs::Empty::ConstPtr& empty);
		void closeCallback(const std_msgs::Empty::ConstPtr& empty);

		void power_on(void);
		void power_off(void);
		void fire(void);
		void my_open(void);
		void my_close(void);


		ros::NodeHandle nh;
		ros::Subscriber poweron_sub;
		ros::Subscriber poweroff_sub;
		ros::Subscriber fire_sub;
		ros::Subscriber open_sub;
		ros::Subscriber close_sub;

		int fd; // file description for the serial port
		struct termios port_settings;      // structure to store the port settings in

};

DriveLauncher::DriveLauncher()
{

	fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);

	if(fd == -1) // if open is unsucessful
	{
		//perror("open_port: Unable to open /dev/ttyS0 - ");
		printf("open_port: Unable to open /dev/ttyACM0. \n");
	}
	else
	{
		fcntl(fd, F_SETFL, 0);
		printf("port is open.\n");
	}


	cfsetispeed(&port_settings, B9600);    // set baud rates
	cfsetospeed(&port_settings, B9600);

	port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;

	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port

	poweron_sub = nh.subscribe<std_msgs::Empty>("/LAUNCHER/power_on", 5, &DriveLauncher::powerOnCallback, this);
	poweroff_sub = nh.subscribe<std_msgs::Empty>("/LAUNCHER/power_off", 5, &DriveLauncher::powerOffCallback, this);
	fire_sub = nh.subscribe<std_msgs::Empty>("/LAUNCHER/fire", 5, &DriveLauncher::fireCallback, this);
	open_sub = nh.subscribe<std_msgs::Empty>("/LAUNCHER/open", 5, &DriveLauncher::openCallback, this);
	close_sub = nh.subscribe<std_msgs::Empty>("/LAUNCHER/close", 5, &DriveLauncher::closeCallback, this);


}


void DriveLauncher::powerOnCallback(const std_msgs::Empty::ConstPtr& empty)
{
	power_on();
}

void DriveLauncher::powerOffCallback(const std_msgs::Empty::ConstPtr& empty)
{
	power_off();
}

void DriveLauncher::fireCallback(const std_msgs::Empty::ConstPtr& empty)
{
	power_on();
	usleep(300000);
	fire();
}

void DriveLauncher::openCallback(const std_msgs::Empty::ConstPtr& empty)
{
	my_open();
}

void DriveLauncher::closeCallback(const std_msgs::Empty::ConstPtr& empty)
{
	my_close();
}




void DriveLauncher::power_on(void)
{
	unsigned char commands[1];
	commands[0] = '+';

	write(fd, commands, 1);  //Send data
}

void DriveLauncher::power_off(void)
{
	unsigned char commands[1];
	commands[0] = '-';

	write(fd, commands, 1);  //Send data
}

void DriveLauncher::fire(void)
{
	unsigned char commands[1];
	commands[0] = 'o';

	write(fd, commands, 1);  //Send data
}

void DriveLauncher::my_open(void)
{
	unsigned char commands[1];
	commands[0] = 'p';

	write(fd, commands, 1);  //Send data
}

void DriveLauncher::my_close(void)
{
	unsigned char commands[1];
	commands[0] = 'c';

	write(fd, commands, 1);  //Send data
}







int main(int argc, char **argv)
{


	ros::init(argc, argv, "Launcher_Driver");
	DriveLauncher drive;
	// Refresh rate
	ros::Rate loop_rate(60);                                // 35 with bluetooth


	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}
