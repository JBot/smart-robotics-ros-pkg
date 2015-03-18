/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
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

#define RADIO_OFF	0
#define RADIO_METAL	1
#define RADIO_JAP	2
#define RADIO_90s	3

#define RADIO_SPECIAL1	10

class nestorRadio {
    public:
        nestorRadio();

        ros::Subscriber radioVolume_sub_;

        ros::Subscriber radioMetal_sub_;
        ros::Subscriber radioJap_sub_;
        ros::Subscriber radio90s_sub_;
        ros::Subscriber radioOff_sub_;
        ros::Subscriber radioSpecial1_sub_;


    private:
	void radioVolumeCallback(const std_msgs::Int32::ConstPtr & feedback);
	void radioMetalCallback(const std_msgs::Empty::ConstPtr & feedback);
	void radioJapCallback(const std_msgs::Empty::ConstPtr & feedback);
	void radio90sCallback(const std_msgs::Empty::ConstPtr & feedback);
	void radioOffCallback(const std_msgs::Empty::ConstPtr & feedback);
	void radioSpecial1Callback(const std_msgs::Empty::ConstPtr & feedback);

	int radioStatus;
	int radioVolume;
        ros::NodeHandle nh;

};


/* Constructor */
nestorRadio::nestorRadio()
{
	radioVolume_sub_ = nh.subscribe < std_msgs::Int32 > ("/NESTOR/radio_volume", 1, &nestorRadio::radioVolumeCallback, this);
	
	radioMetal_sub_ = nh.subscribe < std_msgs::Empty > ("/NESTOR/radio_metal", 1, &nestorRadio::radioMetalCallback, this);
	radioJap_sub_ = nh.subscribe < std_msgs::Empty > ("/NESTOR/radio_jap", 1, &nestorRadio::radioJapCallback, this);
	radio90s_sub_ = nh.subscribe < std_msgs::Empty > ("/NESTOR/radio_90s", 1, &nestorRadio::radio90sCallback, this);
	radioOff_sub_ = nh.subscribe < std_msgs::Empty > ("/NESTOR/radio_off", 1, &nestorRadio::radioOffCallback, this);
	radioSpecial1_sub_ = nh.subscribe < std_msgs::Empty > ("/NESTOR/radio_special1", 1, &nestorRadio::radioSpecial1Callback, this);

	radioStatus = RADIO_OFF;
	radioVolume = 3000;
}


void nestorRadio::radioVolumeCallback(const std_msgs::Int32::ConstPtr & feedback)
{
	if(radioVolume != feedback->data)
	{
		radioVolume = feedback->data;
	}
}

void nestorRadio::radioMetalCallback(const std_msgs::Empty::ConstPtr & feedback)
{
	char my_buff[256];

	switch(radioStatus)
	{
		case RADIO_OFF:
			sprintf(my_buff, "mpg123 -C -f %d http://91.121.75.155:8000 &", radioVolume);
			std::cout << my_buff << std::endl;
			system(my_buff);
			radioStatus = RADIO_METAL;
			break;
		case RADIO_METAL:
			radioStatus = RADIO_METAL;
			break;
		default:
			system("killall mpg123");
			radioStatus = RADIO_METAL;
			usleep(100000);
			sprintf(my_buff, "mpg123 -C -f %d http://91.121.75.155:8000 &", radioVolume);
			std::cout << my_buff << std::endl;
			system(my_buff);
			break;
	}
}

void nestorRadio::radioJapCallback(const std_msgs::Empty::ConstPtr & feedback)
{
	char my_buff[256];

        switch(radioStatus)
        {
                case RADIO_OFF:
			sprintf(my_buff, "mpg123 -C -f %d http://198.27.80.17:8000 &", radioVolume);
			std::cout << my_buff << std::endl;
			system(my_buff);
                        radioStatus = RADIO_JAP;
                        break;
                case RADIO_JAP:
                        radioStatus = RADIO_JAP;
                        break;
                default:
                        system("killall mpg123");
                        radioStatus = RADIO_JAP;
                        usleep(100000);
			sprintf(my_buff, "mpg123 -C -f %d http://198.27.80.17:8000 &", radioVolume);
			std::cout << my_buff << std::endl;
			system(my_buff);
                        break;
        }
}

void nestorRadio::radio90sCallback(const std_msgs::Empty::ConstPtr & feedback)
{
	char my_buff[256];

        switch(radioStatus)
        {
                case RADIO_OFF:
			sprintf(my_buff, "mpg123 -C -f %d http://173.199.116.227:12108 &", radioVolume);
			std::cout << my_buff << std::endl;
			system(my_buff);
                        radioStatus = RADIO_90s;
                        break;
                case RADIO_90s:
                        radioStatus = RADIO_90s;
                        break;
                default:
                        system("killall mpg123");
                        radioStatus = RADIO_90s;
                        usleep(100000);
			sprintf(my_buff, "mpg123 -C -f %d http://173.199.116.227:12108 &", radioVolume);
			std::cout << my_buff << std::endl;
			system(my_buff);
                        break;
        }
}

void nestorRadio::radioSpecial1Callback(const std_msgs::Empty::ConstPtr & feedback)
{
	char my_buff[256];

        switch(radioStatus)
        {
                case RADIO_OFF:
			sprintf(my_buff, "mpg123 -C -f %d /home/jbot/Star_Wars-The_Imperial_March.mp3 &", radioVolume);
			std::cout << my_buff << std::endl;
			system(my_buff);
                        radioStatus = RADIO_SPECIAL1;
                        break;
                case RADIO_SPECIAL1:
                        radioStatus = RADIO_SPECIAL1;
                        break;
                default:
                        system("killall mpg123");
                        radioStatus = RADIO_SPECIAL1;
                        usleep(100000);
			sprintf(my_buff, "mpg123 -C -f %d /home/jbot/Star_Wars-The_Imperial_March.mp3 &", radioVolume);
			std::cout << my_buff << std::endl;
			system(my_buff);
                        break;
        }
}



void nestorRadio::radioOffCallback(const std_msgs::Empty::ConstPtr & feedback)
{
        if(radioStatus != RADIO_OFF)
        {
                system("killall mpg123");
		radioStatus = RADIO_OFF;
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
        ros::init(argc, argv, "Nestor_Radio");
        nestorRadio nestor_radio;
        // Refresh rate
        ros::Rate loop_rate(5); /*  */
        while (ros::ok()) {
                //nestor_twit.check();
                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::Duration(2.0).sleep();

        ros::shutdown();
}

