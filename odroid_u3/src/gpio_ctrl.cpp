#include "ros/ros.h"
#include <std_msgs/Int8.h>


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
#include <stdlib.h>

 /****************************************************************
 * Constants
 ****************************************************************/
 
#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64


class GPIOU3 {
	public:
		GPIOU3();
		void read_io(void);

	private:
		int gpio_set_value(unsigned int gpio, unsigned int value);
		int gpio_get_value(unsigned int gpio, unsigned int *value);
		void gpio1Callback(const std_msgs::Int8::ConstPtr& val);
		void gpio2Callback(const std_msgs::Int8::ConstPtr& val);
		void gpio3Callback(const std_msgs::Int8::ConstPtr& val);
		void gpio4Callback(const std_msgs::Int8::ConstPtr& val);
		void gpio5Callback(const std_msgs::Int8::ConstPtr& val);
		void gpio6Callback(const std_msgs::Int8::ConstPtr& val);
		void gpio7Callback(const std_msgs::Int8::ConstPtr& val);
		void gpio8Callback(const std_msgs::Int8::ConstPtr& val);
		
		ros::NodeHandle nh;
		ros::Subscriber gpio1_sub;
		ros::Subscriber gpio2_sub;
		ros::Subscriber gpio3_sub;
		ros::Subscriber gpio4_sub;
		ros::Subscriber gpio5_sub;
		ros::Subscriber gpio6_sub;
		ros::Subscriber gpio7_sub;
		ros::Subscriber gpio8_sub;

		ros::Publisher gpio9_pub;
		ros::Publisher gpio10_pub;
		ros::Publisher gpio11_pub;
		ros::Publisher gpio12_pub;
		ros::Publisher gpio13_pub;
		ros::Publisher gpio14_pub;
		ros::Publisher gpio15_pub;
		ros::Publisher gpio16_pub;


		char old_value297;
		char old_value298;
		char old_value299;
		char old_value300;
		char old_value301;
		char old_value302;
		char old_value303;
		char old_value304;

};

GPIOU3::GPIOU3()
{

	gpio1_sub = nh.subscribe<std_msgs::Int8>("/odroid/gpio1", 5, &GPIOU3::gpio1Callback, this);
	gpio2_sub = nh.subscribe<std_msgs::Int8>("/odroid/gpio2", 5, &GPIOU3::gpio2Callback, this);
	gpio3_sub = nh.subscribe<std_msgs::Int8>("/odroid/gpio3", 5, &GPIOU3::gpio3Callback, this);
	gpio4_sub = nh.subscribe<std_msgs::Int8>("/odroid/gpio4", 5, &GPIOU3::gpio4Callback, this);
	gpio5_sub = nh.subscribe<std_msgs::Int8>("/odroid/gpio5", 5, &GPIOU3::gpio5Callback, this);
	gpio6_sub = nh.subscribe<std_msgs::Int8>("/odroid/gpio6", 5, &GPIOU3::gpio6Callback, this);
	gpio7_sub = nh.subscribe<std_msgs::Int8>("/odroid/gpio7", 5, &GPIOU3::gpio7Callback, this);
	gpio8_sub = nh.subscribe<std_msgs::Int8>("/odroid/gpio8", 5, &GPIOU3::gpio8Callback, this);

	gpio9_pub = nh.advertise < std_msgs::Int8 > ("/odroid/gpio9", 5);
	gpio10_pub = nh.advertise < std_msgs::Int8 > ("/odroid/gpio10", 5);
	gpio11_pub = nh.advertise < std_msgs::Int8 > ("/odroid/gpio11", 5);
	gpio12_pub = nh.advertise < std_msgs::Int8 > ("/odroid/gpio12", 5);
	gpio13_pub = nh.advertise < std_msgs::Int8 > ("/odroid/gpio13", 5);
	gpio14_pub = nh.advertise < std_msgs::Int8 > ("/odroid/gpio14", 5);
	gpio15_pub = nh.advertise < std_msgs::Int8 > ("/odroid/gpio15", 5);
	gpio16_pub = nh.advertise < std_msgs::Int8 > ("/odroid/gpio16", 5);

	old_value297 = 2;
	old_value298 = 2;
	old_value299 = 2;
	old_value300 = 2;
	old_value301 = 2;
	old_value302 = 2;
	old_value303 = 2;
	old_value304 = 2;
}


/****************************************************************
 * gpio_set_value
 ****************************************************************/
int GPIOU3::gpio_set_value(unsigned int gpio, unsigned int value)
{
	int fd, len;
	char buf[MAX_BUF];
 
	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
		return fd;
	}
 
	if (value)
		write(fd, "1", 2);
	else
		write(fd, "0", 2);
 
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int GPIOU3::gpio_get_value(unsigned int gpio, unsigned int *value)
{
	int fd, len;
	char buf[MAX_BUF];
	char ch;

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}
 
	read(fd, &ch, 1);

	if (ch != '0') {
		*value = 1;
	} else {
		*value = 0;
	}
 
	close(fd);
	return 0;
}

/* Function that read all input IO every X ms */
void GPIOU3::read_io(void)
{
	unsigned int value = 0;
	std_msgs::Int8 pub_value;

	if(gpio_get_value(297, &value)) {
		if(value != old_value297) {
			pub_value.data = value;
			gpio9_pub.publish(pub_value);
			old_value297 = value;
		}
	}

	if(gpio_get_value(298, &value)) {
		if(value != old_value298) {
			pub_value.data = value;
			gpio10_pub.publish(pub_value);
			old_value298 = value;
		}
	}

	if(gpio_get_value(299, &value)) {
		if(value != old_value299) {
			pub_value.data = value;
			gpio11_pub.publish(pub_value);
			old_value299 = value;
		}
	}

	if(gpio_get_value(300, &value)) {
		if(value != old_value300) {
			pub_value.data = value;
			gpio12_pub.publish(pub_value);
			old_value300 = value;
		}
	}

	if(gpio_get_value(301, &value)) {
		if(value != old_value301) {
			pub_value.data = value;
			gpio13_pub.publish(pub_value);
			old_value301 = value;
		}
	}

	if(gpio_get_value(302, &value)) {
		if(value != old_value302) {
			pub_value.data = value;
			gpio14_pub.publish(pub_value);
			old_value302 = value;
		}
	}

	if(gpio_get_value(303, &value)) {
		if(value != old_value303) {
			pub_value.data = value;
			gpio15_pub.publish(pub_value);
			old_value303 = value;
		}
	}

	if(gpio_get_value(304, &value)) {
		if(value != old_value304) {
			pub_value.data = value;
			gpio16_pub.publish(pub_value);
			old_value304 = value;
		}
	}

}

/* Fonction permettant de fixer une nouvelle valeur à la GPIO */
void GPIOU3::gpio1Callback(const std_msgs::Int8::ConstPtr& val)
{
	gpio_set_value(289, val->data);
}

/* Fonction permettant de fixer une nouvelle valeur à la GPIO */
void GPIOU3::gpio2Callback(const std_msgs::Int8::ConstPtr& val)
{
	gpio_set_value(290, val->data);
}

/* Fonction permettant de fixer une nouvelle valeur à la GPIO */
void GPIOU3::gpio3Callback(const std_msgs::Int8::ConstPtr& val)
{
	gpio_set_value(291, val->data);
}

/* Fonction permettant de fixer une nouvelle valeur à la GPIO */
void GPIOU3::gpio4Callback(const std_msgs::Int8::ConstPtr& val)
{
	gpio_set_value(292, val->data);
}

/* Fonction permettant de fixer une nouvelle valeur à la GPIO */
void GPIOU3::gpio5Callback(const std_msgs::Int8::ConstPtr& val)
{
	gpio_set_value(293, val->data);
}

/* Fonction permettant de fixer une nouvelle valeur à la GPIO */
void GPIOU3::gpio6Callback(const std_msgs::Int8::ConstPtr& val)
{
	gpio_set_value(294, val->data);
}

/* Fonction permettant de fixer une nouvelle valeur à la GPIO */
void GPIOU3::gpio7Callback(const std_msgs::Int8::ConstPtr& val)
{
	gpio_set_value(295, val->data);
}

/* Fonction permettant de fixer une nouvelle valeur à la GPIO */
void GPIOU3::gpio8Callback(const std_msgs::Int8::ConstPtr& val)
{
	gpio_set_value(296, val->data);
}




int main(int argc, char **argv)
{


	ros::init(argc, argv, "GPIO_U3_Driver");
	GPIOU3 gpio;
	// Refresh rate
	ros::Rate loop_rate(60);                                // 35 with bluetooth


	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		gpio.read_io();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

