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
#include <sensor_msgs/LaserScan.h>

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




class lightDetector {
    public:
        lightDetector();

        ros::Subscriber laser_sub_;
        ros::Subscriber init_sub_;

	ros::Publisher lightON_pub;
	ros::Publisher lightOFF_pub;

    private:
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
	void initCallback(const std_msgs::Empty::ConstPtr &empty);
        ros::NodeHandle nh;

	sensor_msgs::LaserScan reference;
	int mode;
	int status;
	ros::Time lastTime;
};


/* Constructor */
lightDetector::lightDetector()
{
        std::string name;
        ///nh.param("calendar_name", name, std::string(""));

	laser_sub_ = nh.subscribe < sensor_msgs::LaserScan > ("/scan", 5, &lightDetector::laserCallback, this);
	init_sub_ = nh.subscribe < std_msgs::Empty > ("/HOME/init", 5, &lightDetector::initCallback, this);

	lightON_pub = nh.advertise < std_msgs::Empty > ("/milight/light3ON", 3);
	lightOFF_pub = nh.advertise < std_msgs::Empty > ("/milight/light3OFF", 3);

	mode = 0;
	status = 0; // OFF
	lastTime = ros::Time::now();
}

void lightDetector::initCallback(const std_msgs::Empty::ConstPtr &empty)
{
	mode = 1;
}

void lightDetector::laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	std_msgs::Empty msg;
	if(mode == 0)
	{
		reference = *scan;
		
		for(unsigned int i = 0; i < scan->ranges.size(); i++)
                {
                        if(scan->ranges[i] < 0.05)
                        {
				//reference.ranges[i] = 2.5;
				std::cout << "/i: " << i << " point: " << scan->ranges[i] << " ref: " << reference.ranges[i] << std::endl;
                        }
                }

	}
	else
	{
		int j = 0;
		for(unsigned int i = 0; i < scan->ranges.size(); i++)
		{
			if( (scan->ranges[i] < (reference.ranges[i]-0.3)) && (scan->ranges[i]>0.1))
			{
				std::cout << "i: " << i << " point: " << scan->ranges[i] << " ref: " << reference.ranges[i] << std::endl;
				j++;
				if(j>1)
				{
					if(status == 0)
					{
						lightON_pub.publish(msg);
						status = 1;
					}
					lastTime = ros::Time::now();
					return;
				}
			}
		}
		if( (status == 1) && (ros::Time::now() - lastTime).toSec() > 5)
		{
			
			lightOFF_pub.publish(msg);
			status = 0;
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
        ros::init(argc, argv, "Light_Detector");
        lightDetector light_detector;
        // Refresh rate
        ros::Rate loop_rate(20); /* 5 min */
        while (ros::ok()) {
                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::Duration(2.0).sleep();

        ros::shutdown();
}

