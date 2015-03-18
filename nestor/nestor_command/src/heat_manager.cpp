/* ROS includes */
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <rostweet_msgs/postTweet.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

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




class heatManager {
    public:
        heatManager();

        ros::Subscriber heatON_sub_;

        ros::Subscriber heatOFF_sub_;
    private:
	void heatONCallback(const std_msgs::Empty::ConstPtr &empty);
	void heatOFFCallback(const std_msgs::Empty::ConstPtr &empty);
        ros::NodeHandle nh;

	int mode;
};


/* Constructor */
heatManager::heatManager()
{
        std::string name;
        ///nh.param("calendar_name", name, std::string(""));

	heatON_sub_ = nh.subscribe < std_msgs::Empty > ("/HOME/showerHeatON", 5, &heatManager::heatONCallback, this);
	heatOFF_sub_ = nh.subscribe < std_msgs::Empty > ("/HOME/showerHeatOFF", 5, &heatManager::heatOFFCallback, this);
}

void heatManager::heatONCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("python /home/jbot/ediplug-py/src/examples/ediON.py");
}

void heatManager::heatOFFCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("python /home/jbot/ediplug-py/src/examples/ediOFF.py");
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
        ros::init(argc, argv, "Heat_Manager");
        heatManager heat_manager;
        // Refresh rate
        ros::Rate loop_rate(20); /* 5 min */
        while (ros::ok()) {
                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::Duration(2.0).sleep();

        ros::shutdown();
}

