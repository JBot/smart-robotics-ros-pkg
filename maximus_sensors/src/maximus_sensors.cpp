#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <joy/Joy.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                               // for in-/output
#include <string.h>                              // strcat
#include <fcntl.h>                               // for 'O_RDONLY' deklaration
#include <termios.h>                             // for serial

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>

#include <vector>

//#define BAUDRATE B115200
#define BAUDRATE B9600

// Serial
int ser_fd;
struct termios oldtio, newtio;


sensor_msgs::LaserScan my_laser_scan;

void read_serial_port(char first_input) {

	char my_input[50];
	signed int temp_input;
	char read_flag = 0;

	temp_input = 0;


if(ser_fd != -1) {
	// reset laser scan
	my_laser_scan.ranges.std::vector<float>::clear();
	// set the new values


	read_flag = read(ser_fd, my_input, 1);
	while( (my_input[0] != 'S') ) {
		read_flag = read(ser_fd, my_input, 1);
	}
	ROS_INFO("FIRST INPUT OK \n");


	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	ROS_INFO("VALUE : %f",(float)temp_input);
	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	// << 12 >>
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

	// << 18 >>
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);
	read_flag = read(ser_fd, my_input, 1);
	while((read_flag == 0) || (read_flag == -1)) {
		read_flag = read(ser_fd, my_input, 1);
	}
	temp_input = (temp_input * 10) + (my_input[0] - 48);

	my_laser_scan.ranges.std::vector<float>::push_back((float) temp_input/100);

	temp_input = 0;

}
}




/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

	ser_fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NONBLOCK);
	if( ser_fd == -1)
	{
		printf( " Serial Not Open \n" );
	}
	else
	{
		printf( " Serial Open \n" );
		tcgetattr(ser_fd, &oldtio);                             // Backup old port settings
		memset(&newtio, 0, sizeof(newtio));

		newtio.c_iflag = IGNBRK | IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_cflag = BAUDRATE | CREAD | CS8 | CLOCAL;
		newtio.c_lflag = 0;

		//newtio.c_cc[VTIME]     = 0;  /* inter-character timer unused */
		//newtio.c_cc[VMIN]     = 1;   /* blocking read until 6 chars received */

		tcflush(ser_fd, TCIFLUSH);
		tcsetattr(ser_fd, TCSANOW, &newtio);

		memset(&newtio, 0, sizeof(newtio));
		tcgetattr(ser_fd, &newtio);

		fcntl(ser_fd, F_SETFL, FNDELAY);
	}


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
	ros::init(argc, argv, "maximus_sensors");
	ros::NodeHandle nh;
	// Set a Laser scan sensor for the robot
	ros::Publisher laser_sensor_pub;


	//sensor_msgs::LaserScan my_laser_scan;
	// Set a Laser scan sensor for the robot
	laser_sensor_pub = nh.advertise<sensor_msgs::LaserScan>("laser_scan_sensor", 50);


	my_laser_scan.header.frame_id = "/pml";
	my_laser_scan.header.stamp = ros::Time::now();

	//my_laser_scan.angle_min = -0.5235988;
	my_laser_scan.angle_min = -1.0471976;
	//my_laser_scan.angle_max = 0.5235988;
	my_laser_scan.angle_max = 1.0471976;
	//my_laser_scan.angle_increment = 0.0872665;
	my_laser_scan.angle_increment = 0.1745329;
	// time between measurements [seconds]
	my_laser_scan.time_increment = 0.04;
	// time between scans [seconds]
	my_laser_scan.scan_time = 0.8;
	my_laser_scan.range_min = 0.20;
	my_laser_scan.range_max = 1.50;

	// range data [m] (Note: values < range_min or > range_max should be discarded)
	my_laser_scan.ranges = std::vector<float>();
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.35);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.35);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.35);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);


        tf::TransformBroadcaster br;
        tf::Transform transform;



	laser_sensor_pub.publish(my_laser_scan);


	ros::Rate loop_rate(2); // 35 with bluetooth
	float rotation = 0.0;
	while (ros::ok())
	{


		read_serial_port('S');
		my_laser_scan.header.stamp = ros::Time::now();
		laser_sensor_pub.publish(my_laser_scan);

        transform.setOrigin( tf::Vector3(0.05, 0.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/elevator"));

		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}




