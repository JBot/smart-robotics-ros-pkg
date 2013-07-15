#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
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




class DriveRoboClaw {
  public:
    	DriveRoboClaw();

 
  private:
	void Callback(const geometry_msgs::Twist::ConstPtr& vel);
     	ros::NodeHandle nh;
     	ros::Subscriber vel_sub;
     	ros::Publisher vel_pub;


};

DriveRoboClaw::DriveRoboClaw()
{
	vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &DriveRoboClaw::Callback, this);
	vel_pub = nh.advertise < geometry_msgs::Twist > ("/cmd_vel_out", 3);
}


void DriveRoboClaw::Callback(const geometry_msgs::Twist::ConstPtr& vel)
{
  geometry_msgs::Twist velocity;
  velocity.linear = vel->linear;
  velocity.angular = vel->angular;
  velocity.angular.z = -velocity.angular.z;
  vel_pub.publish(velocity);
}



int main(int argc, char **argv)
{


    ros::init(argc, argv, "RoboClaw_Driver");
    DriveRoboClaw drive;
    // Refresh rate
    ros::Rate loop_rate(60);                                // 35 with bluetooth


    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}

