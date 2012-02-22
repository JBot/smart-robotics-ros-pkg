#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include <sensor_msgs/JointState.h>

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
//#include <math.h>
//#include <algorithm>

#include <vector>



#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>                               // for in-/output
#include <string.h>                              // strcat
#include <fcntl.h>                               // for 'O_RDONLY' deklaration
#include <termios.h>                             // for serial
// For p_thread :
#include <pthread.h>
// For I2C : 
#include <linux/i2c-dev.h>
// For gettimeofday
#include <getopt.h>
#include <sys/time.h>

#define STOCK_HEIGHT	80

#define min(x1,x2) ((x1) > (x2) ? (x2):(x1))
#define max(x1,x2) ((x1) > (x2) ? (x1):(x2))


class indomptableARM_manager {
  public:
    indomptableARM_manager();

    ros::Subscriber pose_sub_;
    ros::Subscriber left_done_sub_;
    ros::Subscriber right_done_sub_;
    ros::Publisher left_pose_pub;
    ros::Publisher right_pose_pub;

    ros::Publisher straight_move_pub;

  private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
    void leftdoneCallback(const std_msgs::Int32::ConstPtr & done);
    void rightdoneCallback(const std_msgs::Int32::ConstPtr & done);

    ros::NodeHandle nh;


    int left_ready;
    int right_ready;


};

indomptableARM_manager::indomptableARM_manager()
{

    pose_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("object_pose", 5, &indomptableARM_manager::poseCallback, this);

    left_done_sub_ = nh.subscribe < std_msgs::Int32 > ("left_arm_done", 5, &indomptableARM_manager::leftdoneCallback, this);
    right_done_sub_ = nh.subscribe < std_msgs::Int32 > ("right_arm_done", 5, &indomptableARM_manager::rightdoneCallback, this);

    left_pose_pub = nh.advertise < geometry_msgs::PoseStamped > ("left_arm_pose", 5);
    right_pose_pub = nh.advertise < geometry_msgs::PoseStamped > ("right_arm_pose", 5);

    straight_move_pub = nh.advertise < std_msgs::Int32 > ("delta_ros", 3);

    left_ready = 1;
    right_ready = 1;



}

void indomptableARM_manager::leftdoneCallback(const std_msgs::Int32::ConstPtr & done) {
	left_ready = done->data;
}

void indomptableARM_manager::rightdoneCallback(const std_msgs::Int32::ConstPtr & done) {
	right_ready = done->data;
}


void indomptableARM_manager::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose)
{

	//usleep(5000000); // For debug only

	geometry_msgs::PoseStamped tmp_pose = *pose;

	if( ((int)(pose->pose.position.y * 1000)) != 0 ) {
	//	takeGround((int)(pose->pose.position.x * 1000), (int)(pose->pose.position.y * 1000));
		if(left_ready != 0) {
			tmp_pose.pose.position.y = -tmp_pose.pose.position.y;
			left_pose_pub.publish(tmp_pose);
			left_ready = 0;
		}
		else if(right_ready != 0) {
			right_pose_pub.publish(tmp_pose);
                        right_ready = 0;
                }

	}
	else {
		if( ((int)(pose->pose.position.z * 1000)) == 0 ) {
	//		takeBARinTotem();
                        left_pose_pub.publish(tmp_pose);
                        left_ready = 0;
		}
		else {
	 //		takeCDinTotem((int)(pose->pose.position.z * 1000));

			if((left_ready != 0) && (right_ready != 0)) {
				left_pose_pub.publish(tmp_pose);
				right_pose_pub.publish(tmp_pose);
				left_ready = 0;
				right_ready = 0;
			}

		}
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
    ros::init(argc, argv, "indomptable_arm_manager");
    indomptableARM_manager indomptablearm_manager;
    // Refresh rate
    ros::Rate loop_rate(10);                                // 35 with bluetooth
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}

