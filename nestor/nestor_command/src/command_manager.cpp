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

#define STOP		10
#define TAKE_PICTURE 	11
#define SEND_MAP 	21

#define NOTHING 	99
#define WAKE_TIME 	0
#define BED_TIME 	1
#define WEATHER 	2
#define LUNCH_TIME 	5



class commandManager {
    public:
        commandManager();

        ros::Subscriber command_sub_;
        ros::Subscriber picture_sub_;
        ros::Subscriber map_sub_;
        //ros::Publisher command_pub;
        
	ros::Publisher french_pub;
	ros::Publisher weather_pub;
	ros::Publisher light1ON_pub;
	ros::Publisher light2ON_pub;
	ros::Publisher light3ON_pub;
	ros::Publisher light4ON_pub;
	ros::Publisher light1OFF_pub;
	ros::Publisher light2OFF_pub;
	ros::Publisher light3OFF_pub;
	ros::Publisher light4OFF_pub;

	ros::ServiceClient twit_client;

	void loop(void);

    private:
	void commandCallback(const std_msgs::Int32::ConstPtr & feedback);
	void pictureCallback(const sensor_msgs::Image::ConstPtr & feedback);
	void mapCallback(const sensor_msgs::Image::ConstPtr & feedback);
        ros::NodeHandle nh;

	int mode;

	ros::Time starting_time;
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

    	french_pub = nh.advertise < std_msgs::String > ("/nestor/french_voice", 3);
    	weather_pub = nh.advertise < std_msgs::Empty > ("/nestor/weather", 3);
    	light1ON_pub = nh.advertise < std_msgs::Empty > ("/milight/light1ON", 3);
    	light2ON_pub = nh.advertise < std_msgs::Empty > ("/milight/light2ON", 3);
    	light3ON_pub = nh.advertise < std_msgs::Empty > ("/milight/light3ON", 3);
    	light4ON_pub = nh.advertise < std_msgs::Empty > ("/milight/light4ON", 3);
    	light1OFF_pub = nh.advertise < std_msgs::Empty > ("/milight/light1OFF", 3);
    	light2OFF_pub = nh.advertise < std_msgs::Empty > ("/milight/light2OFF", 3);
    	light3OFF_pub = nh.advertise < std_msgs::Empty > ("/milight/light3OFF", 3);
    	light4OFF_pub = nh.advertise < std_msgs::Empty > ("/milight/light4OFF", 3);

	//twit_client = nh.serviceClient<rostweet_msgs::postTweet>("/rostweet/postTweet");

	mode = NOTHING;

	starting_time = ros::Time::now();

}

void commandManager::commandCallback(const std_msgs::Int32::ConstPtr & feedback)
{
	std_msgs::String to_send;
	std_msgs::Empty empty_to_send;
	switch(feedback->data) {
		case TAKE_PICTURE :
			//mode = TAKE_PICTURE;	
			ROS_INFO("Take picture command.");		
			break;
		case SEND_MAP :
			//mode = SEND_MAP;	
			ROS_INFO("Send map command.");		
			break;
		case WAKE_TIME :
			if(mode == WAKE_TIME)
			{
			}
			else
			{
				starting_time = ros::Time::now();
				mode = WAKE_TIME;	
				system("/home/jbot/milight_sources/test.sh &");
				ROS_INFO("Send wake command.");	
			}
			break;
		case BED_TIME :
			if(mode == BED_TIME)
			{
			}
			else 
			{
				starting_time = ros::Time::now();
				mode = BED_TIME;	
				to_send.data = "Il est l'heure d'aller dormir";
                        	french_pub.publish(to_send);
				light1ON_pub.publish(empty_to_send);
				light3ON_pub.publish(empty_to_send);
				ROS_INFO("Send sleep command.");
			}
			break;
		case WEATHER :
			//mode = WEATHER;	
                        weather_pub.publish(empty_to_send);
			ROS_INFO("Send weather command.");		
			break;
		case LUNCH_TIME :
			//mode = LUNCH_TIME;	
			to_send.data = "Il est l'heure de se mettre à table";
                        french_pub.publish(to_send);
			ROS_INFO("Send lunch command.");		
			break;
		default:
			ROS_INFO("Unknown command.");
			break;
	}


}

void commandManager::loop(void)
{

	std_msgs::String to_send;
	std_msgs::Empty empty_to_send;

	switch(mode) {
                case TAKE_PICTURE :
                        break;
                case SEND_MAP :
                        break;
                case WAKE_TIME :
			if( (ros::Time::now() - starting_time).toSec() < 120)
			{
			}
			else
			{
				light2ON_pub.publish(empty_to_send);
                        	to_send.data = "Il est l'heure de se lever. Debout les feignants.";
                        	french_pub.publish(to_send);
				mode = NOTHING;
			}
                        break;
                case BED_TIME :
			if( (ros::Time::now() - starting_time).toSec() < (60*30))
                        {
                        } 
                        else
                        {	
				light1OFF_pub.publish(empty_to_send);
				light2OFF_pub.publish(empty_to_send);
				light3OFF_pub.publish(empty_to_send);
				light4OFF_pub.publish(empty_to_send);
				mode = NOTHING;
			}
                        break;
                case WEATHER :
                        break;
                case LUNCH_TIME :
                        break;
                default:
                        break;
        }


}


void commandManager::pictureCallback(const sensor_msgs::Image::ConstPtr & feedback)
{
/*	if(mode == TAKE_PICTURE) {
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
*/
}

void commandManager::mapCallback(const sensor_msgs::Image::ConstPtr & feedback)
{
/*        if(mode == SEND_MAP) {
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
*/

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
        while (ros::ok()) 
	{
                command_manager.loop();
                ros::spinOnce();
                loop_rate.sleep();
        }

        ros::Duration(2.0).sleep();

        ros::shutdown();
}

