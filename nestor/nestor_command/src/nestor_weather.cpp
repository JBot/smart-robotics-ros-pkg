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

class nestorWeather {
    public:
        nestorWeather();

	ros::Publisher english_pub;

        ros::Subscriber weather_sub_;


    private:
	void weatherCallback(const std_msgs::Empty::ConstPtr & feedback);

        ros::NodeHandle nh;

};


/* Constructor */
nestorWeather::nestorWeather()
{
	english_pub = nh.advertise < std_msgs::String > ("nestor/english_voice", 3);

	weather_sub_ = nh.subscribe < std_msgs::Empty > ("/nestor/weather", 2, &nestorWeather::weatherCallback, this);
}


void nestorWeather::weatherCallback(const std_msgs::Empty::ConstPtr & feedback)
{

	FILE *fp;
  	int status;
  	char path[1035];
	std_msgs::String to_send;

	/* Current weather */	
	/* Open the command for reading. */
  	fp = popen("/home/jbot/ROS/smart-robotics-ros-pkg/nestor/nestor_command/current.sh \"EUR|FR|FR|Paris\"", "r");
  	if (fp == NULL) {
    		printf("Failed to run command\n" );
    		exit;
  	}

  	/* Read the output a line at a time - output it. */
  	while (fgets(path, sizeof(path)-1, fp) != NULL) {
    		//printf("%s", path);

		//std::cout << path << std::endl;

		std::string my_string = path;
		int pos = my_string.find(":");

		std::string str1 = my_string.substr(0, pos);

		std::string str3 = my_string.substr(pos+1);
		pos = str3.find("C");
		str3 = str3.substr(1, pos-1);
/*
		std::cout << str1 << std::endl;
		std::cout << str3 << std::endl;
*/
		to_send.data = "Currently it is. ";
		to_send.data += str1;
		to_send.data += ". The temperature is around. ";
		to_send.data += str3;
		to_send.data += ". degrees.";


		english_pub.publish(to_send);
  	}

  	/* close */
  	pclose(fp);


        /* Forecast weather */   
        /* Open the command for reading. */
        fp = popen("/home/jbot/ROS/smart-robotics-ros-pkg/nestor/nestor_command/forecast.sh \"EUR|FR|FR|Paris\"", "r");
        if (fp == NULL) {
                printf("Failed to run command\n" );
                exit;
        }

	to_send.data = "";
	int i = 0;
        /* Read the output a line at a time - output it. */
        while (fgets(path, sizeof(path)-1, fp) != NULL) {
                //printf("%s", path);
                //std::cout << path << std::endl;
		std::string str1;
		std::string str2;
		std::string str3;

		if(i == 0) { /* Today */
			to_send.data += "Today it is going to be. ";
			std::string my_string = path;
						
			int pos = my_string.find("C");
			str3 = my_string.substr(1, pos-1); // Highest

			pos = my_string.find(":");
			my_string =  my_string.substr(pos+1);
			pos = my_string.find("C");
			str2 = my_string.substr(1, pos-1); // Lowest

			str1 = my_string.substr(pos+1);

			to_send.data += str1;
			to_send.data += ". The temperature will be between. ";
			to_send.data += str2;
			to_send.data += " and. ";
			to_send.data += str3;
			to_send.data += " degrees. ";
			i++;
		}
		else { /* Tomorrow */

                        to_send.data += "Tomorrow it is going to be. ";
                        std::string my_string = path;

                        int pos = my_string.find("C");
                        str3 = my_string.substr(1, pos-1); // Highest

                        pos = my_string.find(":");
                        my_string =  my_string.substr(pos+1);
                        pos = my_string.find("C");
                        str2 = my_string.substr(1, pos-1); // Lowest

                        str1 = my_string.substr(pos+1);

                        to_send.data += str1;
                        to_send.data += ". The temperature will be between. ";
                        to_send.data += str2;
                        to_send.data += " and. ";
                        to_send.data += str3;
                        to_send.data += " degrees. ";


		}
        }

	usleep(5000000);
	english_pub.publish(to_send);


        /* close */
        pclose(fp);


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
        ros::init(argc, argv, "Nestor_Weather");
        nestorWeather nestor_weather;
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

