#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
 
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                                         // for in-/output
#include <string.h>                                        // strcat
#include <fcntl.h>                                         // for 'O_RDONLY' deklaration
#include <termios.h>                                       // for serial

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>

#include <vector>



#include <unistd.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>                               // for in-/output
#include <string.h>                              // strcat
#include <fcntl.h>                               // for 'O_RDONLY' deklaration
#include <termios.h>                             // for serial
// For p_thread :
#include <pthread.h>
// For gettimeofday
#include <getopt.h>
#include <sys/time.h>

#define SSCDEVICE "/dev/ttyUSB0"
#define BAUDRATE B115200

// SERVO OFFSET
#define SERVO_OFFSET0   (-88)
#define SERVO_OFFSET4   80
#define SERVO_OFFSET8   194
#define SERVO_OFFSET12   101


class SSC32_ctrl {
  public:
    SSC32_ctrl();

    ros::Subscriber leftcoxa_sub;
    ros::Subscriber leftfemur_sub;
    ros::Subscriber lefttibia_sub;
    ros::Subscriber leftankle_sub;
    ros::Subscriber leftroll_sub;
    ros::Subscriber lefthand_sub;

  private:
    void leftcoxaCallback(const std_msgs::Float64::ConstPtr & value);
    void leftfemurCallback(const std_msgs::Float64::ConstPtr & value);
    void lefttibiaCallback(const std_msgs::Float64::ConstPtr & value);
    void leftankleCallback(const std_msgs::Float64::ConstPtr & value);
    void leftrollCallback(const std_msgs::Float64::ConstPtr & value);
    void lefthandCallback(const std_msgs::Float64::ConstPtr & value);

    ros::NodeHandle nh;

//====================================================================
//[ANGLES]
double leftCoxaAngle;   //Actual Angle of the Right Front Leg
double leftFemurAngle;
double leftTibiaAngle;
double leftAnkleAngle;
double leftRollAngle;
double leftHandAngle;
//--------------------------------------------------------------------
//[VARIABLES]
int ActualGaitSpeed;

// Serial
int ser_fd_ssc;
struct termios oldtio_ssc, newtio_ssc;


};

SSC32_ctrl::SSC32_ctrl() {

	leftcoxa_sub = nh.subscribe < std_msgs::Float64 > ("left_soulder_roll_joint", 5, &SSC32_ctrl::leftcoxaCallback, this);
	leftfemur_sub = nh.subscribe < std_msgs::Float64 > ("left_soulder_lift_joint", 5, &SSC32_ctrl::leftfemurCallback, this);
	lefttibia_sub = nh.subscribe < std_msgs::Float64 > ("left_elbow_joint", 5, &SSC32_ctrl::lefttibiaCallback, this);
	leftankle_sub = nh.subscribe < std_msgs::Float64 > ("left_wrist_joint", 5, &SSC32_ctrl::leftankleCallback, this);
	leftroll_sub = nh.subscribe < std_msgs::Float64 > ("left_hand_joint", 5, &SSC32_ctrl::leftrollCallback, this);
	lefthand_sub = nh.subscribe < std_msgs::Float64 > ("left_hand", 5, &SSC32_ctrl::lefthandCallback, this);


        ser_fd_ssc = open(SSCDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if( ser_fd_ssc == -1)
        {
                printf( " SSC Serial Not Open \n" );
        }
        else
        {
                printf( " SSC Serial Open \n" );
                tcgetattr(ser_fd_ssc, &oldtio_ssc);                             // Backup old port settings
                memset(&newtio_ssc, 0, sizeof(newtio_ssc));

                newtio_ssc.c_iflag = IGNBRK | IGNPAR;
                newtio_ssc.c_oflag = 0;
                newtio_ssc.c_cflag = BAUDRATE | CREAD | CS8 | CLOCAL;
                newtio_ssc.c_lflag = 0;

                tcflush(ser_fd_ssc, TCIFLUSH);
                tcsetattr(ser_fd_ssc, TCSANOW, &newtio_ssc);

                memset(&newtio_ssc, 0, sizeof(newtio_ssc));
                tcgetattr(ser_fd_ssc, &newtio_ssc);

                fcntl(ser_fd_ssc, F_SETFL, FNDELAY);

        }


}
/*
        char Serout[260]={0};
        int temp = 0;

        std_msgs::Float64 tmp;

        temp = (int)( (double)(CoxaAngle* 180.0 / 3.141592 +90)/0.10588238 ) + 650 + SERVO_OFFSET0;
        sprintf(Serout, "%s #%dP%d", Serout, 0, temp);

        temp = (int)( (double)(FemurAngle* 180.0 / 3.141592 +90)/0.10588238 ) + 650 + SERVO_OFFSET4;
        sprintf(Serout, "%s #%dP%d", Serout, 4, temp);

        temp = (int)( (double)(TibiaAngle* 180.0 / 3.141592 +90)/0.10588238 ) + 650 + SERVO_OFFSET8;
        sprintf(Serout, "%s #%dP%d", Serout, 8, temp);

        temp = (int)( (double)(AnkleAngle* 180.0 / 3.141592 +90)/0.10588238 ) + 650 + SERVO_OFFSET12;
        sprintf(Serout, "%s #%dP%d", Serout, 12, temp);

        temp = (int)( (double)(RollAngle* 180.0 / 3.141592 +90)/0.10588238 ) + 650;
        sprintf(Serout, "%s #%dP%d", Serout, 24, temp);

        temp = (int)( (double)(HandAngle* 180.0 / 3.141592 +90)/0.10588238 ) + 650;
        sprintf(Serout, "%s #%dP%d", Serout, 20, temp);


        // Time and <CR>
        sprintf(Serout, "%s T%d\r", Serout, ActualGaitSpeed);

        // write to serial if connected
        if ( ser_fd_ssc ) {
                write(ser_fd_ssc, &Serout, sizeof(Serout));
                printf("%s \n",Serout);
        }
        //printf("%s \n",Serout);
*/


void SSC32_ctrl::leftcoxaCallback(const std_msgs::Float64::ConstPtr & value) {

        char Serout[260]={0};
        int temp = 0;

        std_msgs::Float64 tmp;

        temp = (int)( (double)(value->data* 180.0 / 3.141592 +90)/0.10588238 ) + 650 + SERVO_OFFSET0;
        sprintf(Serout, "%s #%dP%d", Serout, 0, temp);
        // Time and <CR>
        sprintf(Serout, "%s T%d\r", Serout, ActualGaitSpeed);

        // write to serial if connected
        if ( ser_fd_ssc ) {
                write(ser_fd_ssc, &Serout, sizeof(Serout));
                printf("%s \n",Serout);
        }
}

void SSC32_ctrl::leftfemurCallback(const std_msgs::Float64::ConstPtr & value) {

        char Serout[260]={0};
        int temp = 0;

        std_msgs::Float64 tmp;

        temp = (int)( (double)(value->data* 180.0 / 3.141592 +90)/0.10588238 ) + 650 + SERVO_OFFSET4;
        sprintf(Serout, "%s #%dP%d", Serout, 4, temp);
        // Time and <CR>
        sprintf(Serout, "%s T%d\r", Serout, ActualGaitSpeed);

        // write to serial if connected
        if ( ser_fd_ssc ) {
                write(ser_fd_ssc, &Serout, sizeof(Serout));
                printf("%s \n",Serout);
        }
}

void SSC32_ctrl::lefttibiaCallback(const std_msgs::Float64::ConstPtr & value) {

        char Serout[260]={0};
        int temp = 0;

        std_msgs::Float64 tmp;

        temp = (int)( (double)(value->data* 180.0 / 3.141592 +90)/0.10588238 ) + 650 + SERVO_OFFSET8;
        sprintf(Serout, "%s #%dP%d", Serout, 8, temp);
        // Time and <CR>
        sprintf(Serout, "%s T%d\r", Serout, ActualGaitSpeed);

        // write to serial if connected
        if ( ser_fd_ssc ) {
                write(ser_fd_ssc, &Serout, sizeof(Serout));
                printf("%s \n",Serout);
        }
}

void SSC32_ctrl::leftankleCallback(const std_msgs::Float64::ConstPtr & value) {

        char Serout[260]={0};
        int temp = 0;

        std_msgs::Float64 tmp;

        temp = (int)( (double)(value->data* 180.0 / 3.141592 +90)/0.10588238 ) + 650 + SERVO_OFFSET12;
        sprintf(Serout, "%s #%dP%d", Serout, 12, temp);
        // Time and <CR>
        sprintf(Serout, "%s T%d\r", Serout, ActualGaitSpeed);

        // write to serial if connected
        if ( ser_fd_ssc ) {
                write(ser_fd_ssc, &Serout, sizeof(Serout));
                printf("%s \n",Serout);
        }
}

void SSC32_ctrl::leftrollCallback(const std_msgs::Float64::ConstPtr & value) {

        char Serout[260]={0};
        int temp = 0;

        std_msgs::Float64 tmp;

        temp = (int)( (double)(value->data* 180.0 / 3.141592 +90)/0.10588238 ) + 650;
        sprintf(Serout, "%s #%dP%d", Serout, 24, temp);
        // Time and <CR>
        sprintf(Serout, "%s T%d\r", Serout, ActualGaitSpeed);

        // write to serial if connected
        if ( ser_fd_ssc ) {
                write(ser_fd_ssc, &Serout, sizeof(Serout));
                printf("%s \n",Serout);
        }
}

void SSC32_ctrl::lefthandCallback(const std_msgs::Float64::ConstPtr & value) {

        char Serout[260]={0};
        int temp = 0;

        std_msgs::Float64 tmp;

        temp = (int)( (double)(value->data* 180.0 / 3.141592 +90)/0.10588238 ) + 650;
        sprintf(Serout, "%s #%dP%d", Serout, 20, temp);
        // Time and <CR>
        sprintf(Serout, "%s T%d\r", Serout, ActualGaitSpeed);

        // write to serial if connected
        if ( ser_fd_ssc ) {
                write(ser_fd_ssc, &Serout, sizeof(Serout));
                printf("%s \n",Serout);
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
    ros::init(argc, argv, "ssc32_ctrl");
    SSC32_ctrl ssc32_ctrl;
    // Refresh rate
    ros::Rate loop_rate(5);                                // 35 with bluetooth
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}

