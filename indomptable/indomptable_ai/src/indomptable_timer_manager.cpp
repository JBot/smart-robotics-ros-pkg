#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
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


class TimerManager {
    public:
        TimerManager();
        void main_loop(void);

        ros::Subscriber start_sub_;
        ros::Publisher stop_pub;
        ros::Publisher stop_nav_pub;

    private:
        void startCallback(const std_msgs::Empty::ConstPtr & empty);
        ros::NodeHandle nh;

        ros::Time starting_date;
        uint8_t state;
};

TimerManager::TimerManager()
{
    start_sub_ = nh.subscribe < std_msgs::Empty > ("/start_match", 5, &TimerManager::startCallback, this);
    stop_pub = nh.advertise < std_msgs::Empty > ("/stop_match", 5);
    stop_nav_pub = nh.advertise < std_msgs::Empty > ("/pause_nav", 5);

    state = 0;
    starting_date = ros::Time::now();

    usleep(1000000);
}

void TimerManager::startCallback(const std_msgs::Empty::ConstPtr & empty)
{
    //ROS_ERROR("!");
    if(state == 0) {
        starting_date = ros::Time::now();
        state = 1;
        ROS_ERROR("Starting!");
    }
}

void TimerManager::main_loop(void)
{
    //ROS_ERROR("!");
    std_msgs::Empty tmprelease;
    if(state == 1) {
        ros::Time current_date = ros::Time::now();

        if( (current_date.toSec() - starting_date.toSec()) > 89 /* > 90 secondes */) {
            stop_pub.publish(tmprelease);
            stop_nav_pub.publish(tmprelease);
            ROS_ERROR("Stop!");
        }
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
    ros::init(argc, argv, "indomptable_timer_manager");
    TimerManager timer;

    // Refresh rate
    ros::Rate loop_rate(30);
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
        timer.main_loop();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}
