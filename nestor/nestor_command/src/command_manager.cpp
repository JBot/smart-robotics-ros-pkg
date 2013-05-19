/* ROS includes */
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rostweet_msgs/postTweet.h>
#include <std_msgs/Int32.h>

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

#define STOP		0
#define TAKE_PICTURE 	1
#define SEND_MAP 	2



class commandManager {
    public:
        commandManager();

        ros::Subscriber command_sub_;
        ros::Subscriber picture_sub_;
        ros::Subscriber map_sub_;
        //ros::Publisher command_pub;

	ros::ServiceClient twit_client;

    private:
	void commandCallback(const std_msgs::Int32::ConstPtr & feedback);
	void pictureCallback(const sensor_msgs::Image::ConstPtr & feedback);
	void mapCallback(const sensor_msgs::Image::ConstPtr & feedback);
        ros::NodeHandle nh;

	int mode;
};


/* Constructor */
commandManager::commandManager()
{
        std::string name;
        ///nh.param("calendar_name", name, std::string(""));
    //command_pub = nh.advertise < std_msgs::Int32 > ("/calendar/command", 3);

	command_sub_ = nh.subscribe < std_msgs::Int32 > ("/nestor/command", 5, &commandManager::commandCallback, this);
	picture_sub_ = nh.subscribe < sensor_msgs::Image > ("camera/rgb/image_rect_color", 5, &commandManager::pictureCallback, this);
	map_sub_ = nh.subscribe < sensor_msgs::Image > ("map_image/full", 5, &commandManager::mapCallback, this);


	twit_client = nh.serviceClient<rostweet_msgs::postTweet>("/rostweet/postTweet");

	mode = 0;



}

void commandManager::commandCallback(const std_msgs::Int32::ConstPtr & feedback)
{
	switch(feedback->data) {
		case TAKE_PICTURE :
			mode = TAKE_PICTURE;	
			ROS_INFO("Take picture command.");		
			break;
		case SEND_MAP :
			mode = SEND_MAP;	
			ROS_INFO("Send map command.");		
			break;
		default:
			ROS_INFO("Unknown command.");
			break;
	}


}

void commandManager::pictureCallback(const sensor_msgs::Image::ConstPtr & feedback)
{
	if(mode == TAKE_PICTURE) {
		rostweet_msgs::postTweet srv;
  		srv.request.text = "The requested picture : ";
		srv.request.picture.push_back(*feedback);

   		ROS_INFO("Calling service...");
   		if (!twit_client.call(srv))
    			ROS_ERROR("Failed to call service.");
   		else {
        		if (srv.response.result) {
                		ROS_INFO("Posted successfully");
        		} else {
                		ROS_ERROR("Error posting. See rostweet error messages");
        		}
   		}


		mode = 0;
	}

}

void commandManager::mapCallback(const sensor_msgs::Image::ConstPtr & feedback)
{
        if(mode == SEND_MAP) {
                rostweet_msgs::postTweet srv;
                srv.request.text = "The current map : ";
                srv.request.picture.push_back(*feedback);

                ROS_INFO("Calling service...");
                if (!twit_client.call(srv))
                        ROS_ERROR("Failed to call service.");
                else {
                        if (srv.response.result) {
                                ROS_INFO("Posted successfully");
                        } else {
                                ROS_ERROR("Error posting. See rostweet error messages");
                        }
                }


                mode = 0;
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

