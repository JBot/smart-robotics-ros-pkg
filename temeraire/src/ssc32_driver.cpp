#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include "temeraire/SSC32_Servo.h"

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>                                         // for in-/output
#include <string.h>                                        // strcat

//Include system headers
#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>
#include <vector>

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls


// SERVO OFFSET
#define SERVO_OFFSET0   23
#define SERVO_OFFSET1   94
#define SERVO_OFFSET2   0
#define SERVO_OFFSET3   0
#define SERVO_OFFSET4   107
#define SERVO_OFFSET5   85
#define SERVO_OFFSET6   0
#define SERVO_OFFSET7   0
#define SERVO_OFFSET8   -24
#define SERVO_OFFSET9 	113
#define SERVO_OFFSET10  0
#define SERVO_OFFSET11  0
#define SERVO_OFFSET12  0
#define SERVO_OFFSET13  0
#define SERVO_OFFSET14  0
#define SERVO_OFFSET15  0
#define SERVO_OFFSET16  -40
#define SERVO_OFFSET17  -32
#define SERVO_OFFSET18  0
#define SERVO_OFFSET19  0
#define SERVO_OFFSET20  27
#define SERVO_OFFSET21  47
#define SERVO_OFFSET22  0
#define SERVO_OFFSET23  0
#define SERVO_OFFSET24  101
#define SERVO_OFFSET25  -95
#define SERVO_OFFSET26  0
#define SERVO_OFFSET27  0
#define SERVO_OFFSET28  0
#define SERVO_OFFSET29  0
#define SERVO_OFFSET30  0
#define SERVO_OFFSET31  0




class SSC32Driver {
  public:
    	SSC32Driver();
	void test(void);
 
  private:
	void posCallback(const temeraire::SSC32_Servo::ConstPtr& pos);
	void velCallback(const std_msgs::Int32::ConstPtr& vel);
     	ros::NodeHandle nh;
     	ros::Subscriber pos_sub;
     	ros::Subscriber vel_sub;

     	int fd; // file description for the serial port
	struct termios oldtio_ssc, newtio_ssc;

	int offset_tab[32]; 
	int ActualGaitSpeed;

};

SSC32Driver::SSC32Driver()
{

        fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
        if( fd == -1)
        {
                printf( " SSC Serial Not Open \n" );
        }
        else
        {
                printf( " SSC Serial Open \n" );
                tcgetattr(fd, &oldtio_ssc);                             // Backup old port settings
                memset(&newtio_ssc, 0, sizeof(newtio_ssc));

                newtio_ssc.c_iflag = IGNBRK | IGNPAR;
                newtio_ssc.c_oflag = 0;
                newtio_ssc.c_cflag = B115200 | CREAD | CS8 | CLOCAL;
                newtio_ssc.c_lflag = 0;

                tcflush(fd, TCIFLUSH);
                tcsetattr(fd, TCSANOW, &newtio_ssc);

                memset(&newtio_ssc, 0, sizeof(newtio_ssc));
                tcgetattr(fd, &newtio_ssc);

                fcntl(fd, F_SETFL, FNDELAY);

        }






	offset_tab[0] = SERVO_OFFSET0;
        offset_tab[1] = SERVO_OFFSET1;
        offset_tab[2] = SERVO_OFFSET2;
        offset_tab[3] = SERVO_OFFSET3;
        offset_tab[4] = SERVO_OFFSET4;
        offset_tab[5] = SERVO_OFFSET5;
        offset_tab[6] = SERVO_OFFSET6;
        offset_tab[7] = SERVO_OFFSET7;
        offset_tab[8] = SERVO_OFFSET8;
        offset_tab[9] = SERVO_OFFSET9;
        offset_tab[10] = SERVO_OFFSET10;
        offset_tab[11] = SERVO_OFFSET11;
        offset_tab[12] = SERVO_OFFSET12;
        offset_tab[13] = SERVO_OFFSET13;
        offset_tab[14] = SERVO_OFFSET14;
        offset_tab[15] = SERVO_OFFSET15;
        offset_tab[16] = SERVO_OFFSET16;
        offset_tab[17] = SERVO_OFFSET17;
        offset_tab[18] = SERVO_OFFSET18;
        offset_tab[19] = SERVO_OFFSET19;
        offset_tab[20] = SERVO_OFFSET20;
        offset_tab[21] = SERVO_OFFSET21;
        offset_tab[22] = SERVO_OFFSET22;
        offset_tab[23] = SERVO_OFFSET23;
        offset_tab[24] = SERVO_OFFSET24;
        offset_tab[25] = SERVO_OFFSET25;
        offset_tab[26] = SERVO_OFFSET26;
        offset_tab[27] = SERVO_OFFSET27;
        offset_tab[28] = SERVO_OFFSET28;
        offset_tab[29] = SERVO_OFFSET29;
        offset_tab[30] = SERVO_OFFSET30;
        offset_tab[31] = SERVO_OFFSET31;

	ActualGaitSpeed = 200;

	pos_sub = nh.subscribe<temeraire::SSC32_Servo>("TEMERAIRE/ssc32_comm", 5, &SSC32Driver::posCallback, this);
	vel_sub = nh.subscribe<std_msgs::Int32>("TEMERAIRE/ssc32_speed", 5, &SSC32Driver::velCallback, this);




sleep(1);


        //compute_motor_speed(vel->linear.x, vel->linear.y, vel->angular.z);
        char Serout[200]={0};
        int temp = 0;

/*
        for(int i = 0; i < 32; i++)
        {
		temp = (int)( (float)(0 + 90)/0.10588238 ) + 650 + offset_tab[i];
		sprintf(Serout, "%s #%dP%d", Serout, i, temp);
        }
*/

	sprintf(Serout, "%s #%dP%d", Serout, 0, 1500);
        sprintf(Serout, "%s T%d\r", Serout, 500);
        // write to serial if connected
        if ( fd )
	{
                write(fd, &Serout, sizeof(Serout));
		printf("Writing.\n");
	}






}

void SSC32Driver::test(void)
{
	        //compute_motor_speed(vel->linear.x, vel->linear.y, vel->angular.z);
        char Serout[500]={0};
        int temp = 0;


        for(int i = 0; i < 32; i++)
        {
                temp = (int)( (float)(0 + 90)/0.10588238 ) + 650 + offset_tab[i];
                sprintf(Serout, "%s #%dP%d", Serout, i, temp);
        }


        //sprintf(Serout, "%s #%dP%d", Serout, 0, 1500);
        sprintf(Serout, "%s T%d\r", Serout, 100);
        // write to serial if connected
        if ( fd )
        {
                write(fd, &Serout, sizeof(Serout));
//                printf("Writing.\n%s\n", Serout);
        }


}


void SSC32Driver::posCallback(const temeraire::SSC32_Servo::ConstPtr& pos)
{
	//compute_motor_speed(vel->linear.x, vel->linear.y, vel->angular.z);
	char Serout[500]={0};
	int temp = 0;


	for(int i = 0; i < 32; i++)
	{
		if(pos->position[i] > 9999)
		{
		}
		else {
			temp = (int)( (float)(pos->position[i] + 90)/0.10588238 ) + 650 + offset_tab[i];
			sprintf(Serout, "%s #%dP%d", Serout, i, temp);
		}
	}

	sprintf(Serout, "%s T%d\r", Serout, ActualGaitSpeed);

	// write to serial if connected
	if ( fd )
	{
		write(fd, &Serout, sizeof(Serout));
		printf("Writing.\n%s\n", Serout);
	}

}


void SSC32Driver::velCallback(const std_msgs::Int32::ConstPtr& vel)
{
	ActualGaitSpeed = vel->data;
}

int main(int argc, char **argv)
{


	ros::init(argc, argv, "SSC32_Driver");
	SSC32Driver drive;
	// Refresh rate
	ros::Rate loop_rate(20);                                // 35 with bluetooth


	while (ros::ok()) {
		ros::spinOnce();
		//drive.test();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

