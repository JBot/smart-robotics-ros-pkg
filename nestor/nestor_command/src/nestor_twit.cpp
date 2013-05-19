/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <rostweet_msgs/IncomingTweet.h>

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

class nestorTwit {
    public:
        nestorTwit();

	ros::Publisher command_pub;
	ros::Publisher english_pub;

        ros::Subscriber twit_sub_;


    private:
	void twitCallback(const rostweet_msgs::IncomingTweet::ConstPtr & feedback);

        ros::NodeHandle nh;

};


/* Constructor */
nestorTwit::nestorTwit()
{
    	command_pub = nh.advertise < std_msgs::Int32 > ("/twit/command", 3);
	english_pub = nh.advertise < std_msgs::String > ("nestor/english_voice", 3);

	 twit_sub_ = nh.subscribe < rostweet_msgs::IncomingTweet > ("/rostweet/incomingTweet", 50, &nestorTwit::twitCallback, this);
}


void nestorTwit::twitCallback(const rostweet_msgs::IncomingTweet::ConstPtr & feedback)
{
        std::cout << feedback->user << " : " << feedback->tweet << std::endl;
	
	/* If I am saying something ... */
	if ((feedback->user).compare(0,4,"Jbot") == 0) {
        	std::cout << "I am saying something ... ";

		/* ... to my bot */
		if ((feedback->tweet).compare(0,13,"@ROBOT_NESTOR") == 0) {
        		std::cout << " ... to my bot. " << std::endl;

			/* If it is a command */
			if ((feedback->tweet).compare(14,7,"command") == 0) {
				std_msgs::Int32 my_command;
				my_command.data = atoi(((feedback->tweet).substr(22, 2)).c_str());
				command_pub.publish(my_command);
			}
			else { /* else speak */
           			/* This is something to say */
                                std_msgs::String to_send;
                                to_send.data = "From tweeter";
                                english_pub.publish(to_send);
                                usleep(1000000);
                                to_send.data = feedback->tweet;
                                english_pub.publish(to_send);


			}
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
        ros::init(argc, argv, "Nestor_Twit");
        nestorTwit nestor_twit;
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

