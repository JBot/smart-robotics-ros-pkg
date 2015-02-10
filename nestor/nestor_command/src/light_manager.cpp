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




class lightManager {
    public:
        lightManager();

        ros::Subscriber light1ON_sub_;
        ros::Subscriber light2ON_sub_;
        ros::Subscriber light3ON_sub_;
        ros::Subscriber light4ON_sub_;

        ros::Subscriber light1OFF_sub_;
        ros::Subscriber light2OFF_sub_;
        ros::Subscriber light3OFF_sub_;
        ros::Subscriber light4OFF_sub_;

    private:
	void light1ONCallback(const std_msgs::Empty::ConstPtr &empty);
	void light2ONCallback(const std_msgs::Empty::ConstPtr &empty);
	void light3ONCallback(const std_msgs::Empty::ConstPtr &empty);
	void light4ONCallback(const std_msgs::Empty::ConstPtr &empty);
	void light1OFFCallback(const std_msgs::Empty::ConstPtr &empty);
	void light2OFFCallback(const std_msgs::Empty::ConstPtr &empty);
	void light3OFFCallback(const std_msgs::Empty::ConstPtr &empty);
	void light4OFFCallback(const std_msgs::Empty::ConstPtr &empty);
        ros::NodeHandle nh;

	int mode;
};


/* Constructor */
lightManager::lightManager()
{
        std::string name;
        ///nh.param("calendar_name", name, std::string(""));

	light1ON_sub_ = nh.subscribe < std_msgs::Empty > ("/milight/light1ON", 5, &lightManager::light1ONCallback, this);
	light2ON_sub_ = nh.subscribe < std_msgs::Empty > ("/milight/light2ON", 5, &lightManager::light2ONCallback, this);
	light3ON_sub_ = nh.subscribe < std_msgs::Empty > ("/milight/light3ON", 5, &lightManager::light3ONCallback, this);
	light4ON_sub_ = nh.subscribe < std_msgs::Empty > ("/milight/light4ON", 5, &lightManager::light4ONCallback, this);

	light1OFF_sub_ = nh.subscribe < std_msgs::Empty > ("/milight/light1OFF", 5, &lightManager::light1OFFCallback, this);
	light2OFF_sub_ = nh.subscribe < std_msgs::Empty > ("/milight/light2OFF", 5, &lightManager::light2OFFCallback, this);
	light3OFF_sub_ = nh.subscribe < std_msgs::Empty > ("/milight/light3OFF", 5, &lightManager::light3OFFCallback, this);
	light4OFF_sub_ = nh.subscribe < std_msgs::Empty > ("/milight/light4OFF", 5, &lightManager::light4OFFCallback, this);



}

void lightManager::light1ONCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 1 ON");
}

void lightManager::light2ONCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 2 ON");
}

void lightManager::light3ONCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 3 ON");
}

void lightManager::light4ONCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 4 ON");
}

void lightManager::light1OFFCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 1 OFF");
}

void lightManager::light2OFFCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 2 OFF");
}

void lightManager::light3OFFCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 3 OFF");
}

void lightManager::light4OFFCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 4 OFF");
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
        ros::init(argc, argv, "Command_Manager");
        commandManager command_manager;
        // Refresh rate
        ros::Rate loop_rate(20); /* 5 min */
        while (ros::ok()) {
                //calendarcheck.check_calendar();
                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::Duration(2.0).sleep();

        ros::shutdown();
}

