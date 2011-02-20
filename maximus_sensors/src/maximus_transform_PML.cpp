#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <maximus_sensors/AvrLaserVector.h>
#include <std_msgs/Int32.h>


#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                               // for in-/output
#include <string.h>                              // strcat

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>

#include <vector>


class TransformPML
{
	public:
		TransformPML();
		void publish_all(void);

		// avr suscriber
		ros::Subscriber avrpml_sub_;
		// Set a Laser scan sensor for the robot
		ros::Publisher laser_sensor_pub;

		// avr right suscriber
		ros::Subscriber avrpmlright_sub_;
		// Set a Laser scan sensor for the robot
		ros::Publisher laser_sensor_right_pub;

		// avr left suscriber
		ros::Subscriber avrpmlleft_sub_;
		// Set a Laser scan sensor for the robot
		ros::Publisher laser_sensor_left_pub;

	private:
		void pmlCallback(const maximus_sensors::AvrLaserVector::ConstPtr& pml);
		void rightpmlCallback(const std_msgs::Int32::ConstPtr& pml);
		void leftpmlCallback(const std_msgs::Int32::ConstPtr& pml);
		ros::NodeHandle nh;

		sensor_msgs::LaserScan my_laser_scan;
		sensor_msgs::LaserScan my_laser_scan_right;
		sensor_msgs::LaserScan my_laser_scan_left;
};

TransformPML::TransformPML()
{

	// Joystick suscriber
	avrpml_sub_ = nh.subscribe<maximus_sensors::AvrLaserVector>("responseavrlaservector", 10, &TransformPML::pmlCallback, this);
	// Set a Laser scan sensor for the robot
	laser_sensor_pub = nh.advertise<sensor_msgs::LaserScan>("avr_PML_sensor", 10);

	// Joystick suscriber
	avrpmlright_sub_ = nh.subscribe<std_msgs::Int32>("rightscan", 10, &TransformPML::rightpmlCallback, this);
	// Set a Laser scan sensor for the robot
	laser_sensor_right_pub = nh.advertise<sensor_msgs::LaserScan>("avr_PML_sensor_right", 10);

	// Joystick suscriber
	avrpmlleft_sub_ = nh.subscribe<std_msgs::Int32>("leftscan", 10, &TransformPML::leftpmlCallback, this);
	// Set a Laser scan sensor for the robot
	laser_sensor_left_pub = nh.advertise<sensor_msgs::LaserScan>("avr_PML_sensor_left", 10);

	my_laser_scan.header.frame_id = "/elevator";
	my_laser_scan.header.stamp = ros::Time::now();

	my_laser_scan.angle_min = (75 * 3.14159265 / 180) - (3.14159265 / 2);
	my_laser_scan.angle_max = (111 * 3.14159265 / 180) - (3.14159265 / 2);
	my_laser_scan.angle_increment = (2 * 3.14159265 / 180);
	// time between measurements [seconds]
	my_laser_scan.time_increment = 0.04;
	// time between scans [seconds]
	my_laser_scan.scan_time = 0.1;
	my_laser_scan.range_min = 0.20;
	my_laser_scan.range_max = 1.50;

	// range data [m] (Note: values < range_min or > range_max should be discarded)
	my_laser_scan.ranges = std::vector<float>();
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.15);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.35);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.15);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.35);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);

	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.15);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.35);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.15);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.35);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.15);

	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.15);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.35);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.25);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.15);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.35);
	my_laser_scan.ranges.std::vector<float>::push_back((float) 0.15);


	my_laser_scan_left.header.frame_id = "/elevator";
        my_laser_scan_left.header.stamp = ros::Time::now();

        my_laser_scan_left.angle_min = (179 * 3.14159265 / 180) - (3.14159265 / 2);
        my_laser_scan_left.angle_max = (180 * 3.14159265 / 180) - (3.14159265 / 2);
        my_laser_scan_left.angle_increment = (1 * 3.14159265 / 180);
        // time between measurements [seconds]
        my_laser_scan_left.time_increment = 0.001;
        // time between scans [seconds]
        my_laser_scan_left.scan_time = 0.1;
        my_laser_scan_left.range_min = 0.15;
        my_laser_scan_left.range_max = 0.80;

        // range data [m] (Note: values < range_min or > range_max should be discarded)
        my_laser_scan_left.ranges = std::vector<float>();
        my_laser_scan_left.ranges.std::vector<float>::push_back((float) 0.15);
        my_laser_scan_left.ranges.std::vector<float>::push_back((float) 0.35);

        my_laser_scan_right.header.frame_id = "/elevator";
        my_laser_scan_right.header.stamp = ros::Time::now();

        my_laser_scan_right.angle_min = (0 * 3.14159265 / 180) - (3.14159265 / 2);
        my_laser_scan_right.angle_max = (1 * 3.14159265 / 180) - (3.14159265 / 2);
        my_laser_scan_right.angle_increment = (1 * 3.14159265 / 180);
        // time between measurements [seconds]
        my_laser_scan_right.time_increment = 0.001;
        // time between scans [seconds]
        my_laser_scan_right.scan_time = 0.1;
        my_laser_scan_right.range_min = 0.15;
        my_laser_scan_right.range_max = 0.80;

        // range data [m] (Note: values < range_min or > range_max should be discarded)
        my_laser_scan_right.ranges = std::vector<float>();
        my_laser_scan_right.ranges.std::vector<float>::push_back((float) 0.15);
        my_laser_scan_right.ranges.std::vector<float>::push_back((float) 0.35);

}


void TransformPML::rightpmlCallback(const std_msgs::Int32::ConstPtr& pml)
{
	my_laser_scan_right.ranges.std::vector<float>::clear();
	my_laser_scan_right.ranges.std::vector<float>::push_back((float) (((float)pml->data)/100.0 + 0.15));
	my_laser_scan_right.ranges.std::vector<float>::push_back((float) (((float)pml->data)/100.0 + 0.15));

	my_laser_scan_right.header.stamp = ros::Time::now();
	laser_sensor_right_pub.publish(my_laser_scan_right);
}

void TransformPML::leftpmlCallback(const std_msgs::Int32::ConstPtr& pml)
{
        my_laser_scan_left.ranges.std::vector<float>::clear();
        my_laser_scan_left.ranges.std::vector<float>::push_back((float) (((float)pml->data)/100.0 + 0.15));
        my_laser_scan_left.ranges.std::vector<float>::push_back((float) (((float)pml->data)/100.0 + 0.15));

        my_laser_scan_left.header.stamp = ros::Time::now();
        laser_sensor_left_pub.publish(my_laser_scan_left);

}


void TransformPML::pmlCallback(const maximus_sensors::AvrLaserVector::ConstPtr& pml)
{
	float tmp = 0.0;

	my_laser_scan.ranges.std::vector<float>::clear();
	tmp = pml->ranges[17] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[16] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[15] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[14] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[13] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[12] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);

	tmp = pml->ranges[11] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[10] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[9] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[8] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[7] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[6] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);

	tmp = pml->ranges[5] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[4] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[3] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[2] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[1] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);
	tmp = pml->ranges[0] / 100;
	my_laser_scan.ranges.std::vector<float>::push_back(tmp);

	my_laser_scan.header.stamp = ros::Time::now();
	laser_sensor_pub.publish(my_laser_scan);

}


void TransformPML::publish_all(void) {

	// Publish the laser scan values
	TransformPML::laser_sensor_pub.publish(my_laser_scan);
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
	ros::init(argc, argv, "transformPML");
	TransformPML transform_PML;
	// Refresh rate
	ros::Rate loop_rate(5); // 35 with bluetooth
	while (ros::ok())
	{
		// Publish all the values and messages
		//maximus_talker.publish_all();
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}




