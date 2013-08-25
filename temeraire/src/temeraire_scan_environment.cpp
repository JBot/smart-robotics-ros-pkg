#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

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

/************ DEFINES ***********/

#define False 0
#define True  1 
#define FALSE 0
#define TRUE  1

#define START_ANGLE	-0.24
#define STOP_ANGLE	0.24



class TemeraireScanner {
	public:
		TemeraireScanner();
		void computeNextStep(void);

	private:
		void scanCallback(const std_msgs::Empty::ConstPtr& vel);


		ros::NodeHandle nh;
		ros::Subscriber start_scan_sub;

		ros::Publisher pose_pub;
		ros::Publisher mode_pub;


		int starting, direction;
		double rotation_angle;
		

};

TemeraireScanner::TemeraireScanner()
{
	start_scan_sub = nh.subscribe<std_msgs::Empty>("/TEMERAIRE/start_scan", 5, &TemeraireScanner::scanCallback, this);

	pose_pub = nh.advertise < geometry_msgs::Twist > ("/TEMERAIRE/body_pose", 3);
	mode_pub = nh.advertise < std_msgs::Int32 > ("/TEMERAIRE/mode", 3);


	direction = 0; // 0 = Y, 1 = X
	starting = 0;
	rotation_angle = START_ANGLE;
}


void TemeraireScanner::scanCallback(const std_msgs::Empty::ConstPtr& vel)
{

	starting = 1;
	direction = 0; // 0 = Y, 1 = X
	rotation_angle = START_ANGLE;

}


void TemeraireScanner::computeNextStep(void)
{
	if(starting == 1) {
		
		geometry_msgs::Twist twist;
		twist.linear.x = 0.0;
		twist.linear.y = 0.0;
		twist.linear.z = 0.0;
		twist.angular.x = 0.0;
		twist.angular.y = 0.0;
		twist.angular.z = 0.0;

		pose_pub.publish(twist);

		starting++;
	}
	else if( starting == 2) {
		starting++;
	}
	else if( starting == 3) {
		std_msgs::Int32 integer_mode;
		integer_mode.data = 1;
		mode_pub.publish(integer_mode);

		starting++;
	}
	else if( (starting > 3) && (starting < 100)) {
                geometry_msgs::Twist twist;
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.linear.z = 0.0;
		if(direction == 1) {
                	twist.angular.x = rotation_angle;
                	twist.angular.y = 0.0;
			
			if(rotation_angle < STOP_ANGLE) {
				rotation_angle = rotation_angle + 0.02;
			}
			else {
				rotation_angle = START_ANGLE;
				starting = 100;
				direction = 0;
			}
		}
		else {
                	twist.angular.x = 0.0;
                	twist.angular.y = rotation_angle;

			if(rotation_angle < STOP_ANGLE) {
                                rotation_angle = rotation_angle + 0.02;
                        }
                        else {
                                rotation_angle = START_ANGLE;
                                direction = 1;
                        }


		}
                twist.angular.z = 0.0;

		if(rotation_angle == START_ANGLE)
			ros::Duration(0.5).sleep();

                pose_pub.publish(twist);
        }

	


        else if( starting == 100) {
                geometry_msgs::Twist twist;
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.linear.z = 0.0;
                twist.angular.x = 0.0;
                twist.angular.y = 0.0;
                twist.angular.z = 0.0;

                pose_pub.publish(twist);

                starting++;
	
        }
        else if( starting == 101) {
                std_msgs::Int32 integer_mode;
                integer_mode.data = 0;
                mode_pub.publish(integer_mode);
		starting = 0;
        }

}

int main(int argc, char **argv)
{


	ros::init(argc, argv, "Temeraire_Scanner");
	TemeraireScanner temeraire_scanner;
	// Refresh rate
	ros::Rate loop_rate(2);                                // 35 with bluetooth


	while (ros::ok()) {
		ros::spinOnce();
		temeraire_scanner.computeNextStep();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

