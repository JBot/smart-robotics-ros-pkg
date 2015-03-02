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

        ros::Subscriber light1Color_sub_;
        ros::Subscriber light2Color_sub_;
        ros::Subscriber light3Color_sub_;
        ros::Subscriber light4Color_sub_;

        ros::Subscriber light1Brightness_sub_;
        ros::Subscriber light2Brightness_sub_;
        ros::Subscriber light3Brightness_sub_;
        ros::Subscriber light4Brightness_sub_;

        ros::Subscriber light1White_sub_;
        ros::Subscriber light2White_sub_;
        ros::Subscriber light3White_sub_;
        ros::Subscriber light4White_sub_;
    private:
	void light1ONCallback(const std_msgs::Empty::ConstPtr &empty);
	void light2ONCallback(const std_msgs::Empty::ConstPtr &empty);
	void light3ONCallback(const std_msgs::Empty::ConstPtr &empty);
	void light4ONCallback(const std_msgs::Empty::ConstPtr &empty);
	void light1OFFCallback(const std_msgs::Empty::ConstPtr &empty);
	void light2OFFCallback(const std_msgs::Empty::ConstPtr &empty);
	void light3OFFCallback(const std_msgs::Empty::ConstPtr &empty);
	void light4OFFCallback(const std_msgs::Empty::ConstPtr &empty);
	void light1ColorCallback(const std_msgs::Int32::ConstPtr &value);
	void light2ColorCallback(const std_msgs::Int32::ConstPtr &value);
	void light3ColorCallback(const std_msgs::Int32::ConstPtr &value);
	void light4ColorCallback(const std_msgs::Int32::ConstPtr &value);
	void light1BrightnessCallback(const std_msgs::Int32::ConstPtr &value);
	void light2BrightnessCallback(const std_msgs::Int32::ConstPtr &value);
	void light3BrightnessCallback(const std_msgs::Int32::ConstPtr &value);
	void light4BrightnessCallback(const std_msgs::Int32::ConstPtr &value);
	void light1WhiteCallback(const std_msgs::Empty::ConstPtr &empty);
	void light2WhiteCallback(const std_msgs::Empty::ConstPtr &empty);
	void light3WhiteCallback(const std_msgs::Empty::ConstPtr &empty);
	void light4WhiteCallback(const std_msgs::Empty::ConstPtr &empty);
        ros::NodeHandle nh;

	int mode;
};


/* Constructor */
lightManager::lightManager()
{
        std::string name;
        ///nh.param("calendar_name", name, std::string(""));

	light1ON_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light1ON", 5, &lightManager::light1ONCallback, this);
	light2ON_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light2ON", 5, &lightManager::light2ONCallback, this);
	light3ON_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light3ON", 5, &lightManager::light3ONCallback, this);
	light4ON_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light4ON", 5, &lightManager::light4ONCallback, this);

	light1OFF_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light1OFF", 5, &lightManager::light1OFFCallback, this);
	light2OFF_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light2OFF", 5, &lightManager::light2OFFCallback, this);
	light3OFF_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light3OFF", 5, &lightManager::light3OFFCallback, this);
	light4OFF_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light4OFF", 5, &lightManager::light4OFFCallback, this);

	light1Color_sub_ = nh.subscribe < std_msgs::Int32 > ("/MILIGHT/light1Color", 5, &lightManager::light1ColorCallback, this);
	light2Color_sub_ = nh.subscribe < std_msgs::Int32 > ("/MILIGHT/light2Color", 5, &lightManager::light2ColorCallback, this);
	light3Color_sub_ = nh.subscribe < std_msgs::Int32 > ("/MILIGHT/light3Color", 5, &lightManager::light3ColorCallback, this);
	light4Color_sub_ = nh.subscribe < std_msgs::Int32 > ("/MILIGHT/light4Color", 5, &lightManager::light4ColorCallback, this);

	light1Brightness_sub_ = nh.subscribe < std_msgs::Int32 > ("/MILIGHT/light1Brightness", 5, &lightManager::light1BrightnessCallback, this);
	light2Brightness_sub_ = nh.subscribe < std_msgs::Int32 > ("/MILIGHT/light2Brightness", 5, &lightManager::light2BrightnessCallback, this);
	light3Brightness_sub_ = nh.subscribe < std_msgs::Int32 > ("/MILIGHT/light3Brightness", 5, &lightManager::light3BrightnessCallback, this);
	light4Brightness_sub_ = nh.subscribe < std_msgs::Int32 > ("/MILIGHT/light4Brightness", 5, &lightManager::light4BrightnessCallback, this);

	light1White_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light1White", 5, &lightManager::light1WhiteCallback, this);
	light2White_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light2White", 5, &lightManager::light2WhiteCallback, this);
	light3White_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light3White", 5, &lightManager::light3WhiteCallback, this);
	light4White_sub_ = nh.subscribe < std_msgs::Empty > ("/MILIGHT/light4White", 5, &lightManager::light4WhiteCallback, this);
}

void lightManager::light1ONCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 1 ON");
	system("/home/jbot/milight_sources/milight 1 ON");
}

void lightManager::light2ONCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 2 ON");
	system("/home/jbot/milight_sources/milight 2 ON");
}

void lightManager::light3ONCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 3 ON");
	system("/home/jbot/milight_sources/milight 3 ON");
}

void lightManager::light4ONCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 4 ON");
	system("/home/jbot/milight_sources/milight 4 ON");
}

void lightManager::light1OFFCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 1 OFF");
	system("/home/jbot/milight_sources/milight 1 OFF");
}

void lightManager::light2OFFCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 2 OFF");
	system("/home/jbot/milight_sources/milight 2 OFF");
}

void lightManager::light3OFFCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 3 OFF");
	system("/home/jbot/milight_sources/milight 3 OFF");
}

void lightManager::light4OFFCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 4 OFF");
	system("/home/jbot/milight_sources/milight 4 OFF");
}

void lightManager::light1ColorCallback(const std_msgs::Int32::ConstPtr &value)
{
	char my_buff[256];
        sprintf(my_buff, "/home/jbot/milight_sources/milight 1 c %d", value->data);
	std::cout << my_buff << std::endl;
        system(my_buff);
        system(my_buff);
}

void lightManager::light2ColorCallback(const std_msgs::Int32::ConstPtr &value)
{
	char my_buff[256];
        sprintf(my_buff, "/home/jbot/milight_sources/milight 2 c %d", value->data);
	std::cout << my_buff << std::endl;
        system(my_buff);
        system(my_buff);
}

void lightManager::light3ColorCallback(const std_msgs::Int32::ConstPtr &value)
{
	char my_buff[256];
        sprintf(my_buff, "/home/jbot/milight_sources/milight 3 c %d", value->data);
	std::cout << my_buff << std::endl;
        system(my_buff);
        system(my_buff);
}

void lightManager::light4ColorCallback(const std_msgs::Int32::ConstPtr &value)
{
	char my_buff[256];
        sprintf(my_buff, "/home/jbot/milight_sources/milight 4 c %d", value->data);
	std::cout << my_buff << std::endl;
        system(my_buff);
        system(my_buff);
}

void lightManager::light1BrightnessCallback(const std_msgs::Int32::ConstPtr &value)
{
	char my_buff[256];
        sprintf(my_buff, "/home/jbot/milight_sources/milight 1 B %d", value->data);
	std::cout << my_buff << std::endl;
        system(my_buff);
        system(my_buff);
}

void lightManager::light2BrightnessCallback(const std_msgs::Int32::ConstPtr &value)
{
	char my_buff[256];
        sprintf(my_buff, "/home/jbot/milight_sources/milight 2 B %d", value->data);
	std::cout << my_buff << std::endl;
        system(my_buff);
        system(my_buff);
}

void lightManager::light3BrightnessCallback(const std_msgs::Int32::ConstPtr &value)
{
	char my_buff[256];
        sprintf(my_buff, "/home/jbot/milight_sources/milight 3 B %d", value->data);
	std::cout << my_buff << std::endl;
        system(my_buff);
        system(my_buff);
}

void lightManager::light4BrightnessCallback(const std_msgs::Int32::ConstPtr &value)
{
	char my_buff[256];
        sprintf(my_buff, "/home/jbot/milight_sources/milight 4 B %d", value->data);
	std::cout << my_buff << std::endl;
        system(my_buff);
        system(my_buff);
}

void lightManager::light1WhiteCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 1 W");
	system("/home/jbot/milight_sources/milight 1 W");
}

void lightManager::light2WhiteCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 2 W");
	system("/home/jbot/milight_sources/milight 2 W");
}

void lightManager::light3WhiteCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 3 W");
	system("/home/jbot/milight_sources/milight 3 W");
}

void lightManager::light4WhiteCallback(const std_msgs::Empty::ConstPtr &empty)
{
	system("/home/jbot/milight_sources/milight 4 W");
	system("/home/jbot/milight_sources/milight 4 W");
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
        ros::init(argc, argv, "Light_Manager");
        lightManager light_manager;
        // Refresh rate
        ros::Rate loop_rate(20); /* 5 min */
        while (ros::ok()) {
                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::Duration(2.0).sleep();

        ros::shutdown();
}

