/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

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

class nestorVoice {
    public:
        nestorVoice();

        ros::Subscriber french_sub_;
        ros::Subscriber english_sub_;


    private:
	void frenchCallback(const std_msgs::String::ConstPtr & feedback);
	void englishCallback(const std_msgs::String::ConstPtr & feedback);

        ros::NodeHandle nh;

};


/* Constructor */
nestorVoice::nestorVoice()
{
        std::string name;
        ///nh.param("calendar_name", name, std::string(""));
    //command_pub = nh.advertise < std_msgs::Int32 > ("/calendar/command", 3);

	 french_sub_ = nh.subscribe < std_msgs::String > ("nestor/french_voice", 5, &nestorVoice::frenchCallback, this);
	 english_sub_ = nh.subscribe < std_msgs::String > ("nestor/english_voice", 5, &nestorVoice::englishCallback, this);
}


void nestorVoice::frenchCallback(const std_msgs::String::ConstPtr & feedback)
{
	char my_buff[256];
        //cout << buff;
	//strncpy(buff, (feedback->data).c_str(), (feedback->data).size());
	char *buff=new char[(feedback->data).size()+1];
        buff[(feedback->data).size()] = 0;
        memcpy(buff, (feedback->data).c_str(), (feedback->data).size());
	//std::cout << feedback->data << std::endl;
	//buff << feedback->data << std::endl;
        //sprintf(my_buff, "espeak -g 1 -v fr-mbrola-1 \" %s \"", buff);
        sprintf(my_buff, "espeak -g 1 -v mb/mb-fr1 \" %s \"", buff);
        std::cout << my_buff << std::endl;
        system(my_buff);

}

void nestorVoice::englishCallback(const std_msgs::String::ConstPtr & feedback)
{
        char my_buff[256];
        //cout << buff;
        //buff = (feedback->data).c_str();
        //strncpy(buff, (feedback->data).c_str(), (feedback->data).size());
        char *buff=new char[(feedback->data).size()+1];
        buff[(feedback->data).size()] = 0;
        memcpy(buff, (feedback->data).c_str(), (feedback->data).size());
	//std::cout << feedback->data << std::endl;
        //buff << feedback->data << std::endl;
        //sprintf(my_buff, "espeak -g 1 -v us-mbrola-1 \" %s \"", buff);
        sprintf(my_buff, "espeak -g 1 -v mb/mb-us2 \" %s \"", buff);
        std::cout << my_buff << std::endl;
        system(my_buff);

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
        ros::init(argc, argv, "Nestor_Voice");
        nestorVoice nestor_voice;
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

