#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

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



class FireFinder {
    public:
        FireFinder();

        ros::Subscriber laser_opponent_sub_;

        ros::Publisher laser_opp_pub;



    private:
        void laserOppCallback(const sensor_msgs::LaserScan::ConstPtr & laser);
        ros::NodeHandle nh;

	sensor_msgs::LaserScan save_opp_laser;
	int cpt;

        int number_remove;
        double range_min;
};

FireFinder::FireFinder()
{
    ros::NodeHandle nhp("~");

    number_remove = 5;
    range_min = 0.1;

    nhp.getParam("number_remove", number_remove);
    nhp.getParam("range_min", range_min);

    laser_opponent_sub_ = nh.subscribe < sensor_msgs::LaserScan > ("/CARRY/laser_nav", 1, &FireFinder::laserOppCallback, this);
    laser_opp_pub = nh.advertise < sensor_msgs::LaserScan > ("/CARRY/laser_nav2", 5);

    cpt = 0;
}

void FireFinder::laserOppCallback(const sensor_msgs::LaserScan::ConstPtr & laser)
{
	save_opp_laser = *laser;

	int i = 0;
	for(i = ( (save_opp_laser.ranges).size()/2 -number_remove); i < ( (save_opp_laser.ranges).size()/2 +number_remove); i++) {
		save_opp_laser.ranges[i] = 0.001;
	}

	//save_opp_laser = *laser;
	save_opp_laser.range_min = range_min;

	laser_opp_pub.publish(save_opp_laser);

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
    ros::init(argc, argv, "Fire_Finder");

    FireFinder firefinder;

    // Refresh rate
    ros::Rate loop_rate(60);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}
