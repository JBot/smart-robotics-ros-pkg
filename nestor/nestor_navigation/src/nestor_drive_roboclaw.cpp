#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>


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


#define theta1 (3.14159265359 - 2.09439510239)
#define theta2 (3.14159265359)
#define theta3 (3.14159265359 + 2.09439510239)
#define R (0.3)
#define RAYON (0.05)



class DriveRoboClaw {
  public:
    	DriveRoboClaw();
    	void rotate(int32_t speed);

	void write_RoboClaw_forward_M1(char addr, int32_t speed);
	void write_RoboClaw_backward_M1(char addr, int32_t speed);
	void write_RoboClaw_forward_M2(char addr, int32_t speed);
	void write_RoboClaw_backward_M2(char addr, int32_t speed);
	void write_RoboClaw_drive_M1(char addr, int32_t speed);
	void write_RoboClaw_drive_M2(char addr, int32_t speed);
	void write_RoboClaw_speed_M1(char addr, int32_t speed);
	void write_RoboClaw_speed_M2(char addr, int32_t speed);
	void write_RoboClaw_speed_M1M2(char addr, int32_t speedM1, int32_t speedM2);
	void write_RoboClaw_speed_dist_M1M2(char addr, int32_t speedM1, int32_t distanceM1, int32_t speedM2, int32_t distanceM2);
 
  private:
	void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
	void compute_motor_speed(double x, double y, double z);
     	ros::NodeHandle nh;
     	ros::Subscriber vel_sub;

     	int fd; // file description for the serial port
     	struct termios port_settings;      // structure to store the port settings in

	double speed_motor1;
	double speed_motor2;
	double speed_motor3;

};

DriveRoboClaw::DriveRoboClaw()
{

        fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);

        if(fd == -1) // if open is unsucessful
        {
                //perror("open_port: Unable to open /dev/ttyS0 - ");
                printf("open_port: Unable to open /dev/ttyUSB0. \n");
        }
        else
        {
                fcntl(fd, F_SETFL, 0);
                printf("port is open.\n");
        }


        cfsetispeed(&port_settings, B38400);    // set baud rates
        cfsetospeed(&port_settings, B38400);

        port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
        port_settings.c_cflag &= ~CSTOPB;
        port_settings.c_cflag &= ~CSIZE;
        port_settings.c_cflag |= CS8;

        tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port

	speed_motor1 = 0;
	speed_motor2 = 0;
	speed_motor3 = 0;

	vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &DriveRoboClaw::velCallback, this);

}


void DriveRoboClaw::velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  compute_motor_speed(vel->linear.x, vel->linear.y, vel->angular.z);
/*
  write_RoboClaw_speed_M1M2(128, int32_t(speed_motor1 * 20000), int32_t(speed_motor2 * 20000));
  write_RoboClaw_speed_M1(129, int32_t(speed_motor3 * 20000));  
*/

// TODO : brider la vitesse des moteurs
// TODO : SI un moteur va trop vite, prendre son ratio par rapport a vitesse max et l'appliquer aux autres moteurs

double ratio = 1.0;

if( fabs(speed_motor1  * 12.85 ) > fabs(speed_motor2 * 12.85) )
{
	if( fabs(speed_motor1 * 12.85) > fabs(speed_motor3 * 12.85) )
	{ // speed_motor1 is the biggest
		if( fabs(speed_motor1 * 12.85) > 63.0 )
			ratio = fabs(speed_motor1 * 12.85) / 63.0;
	}
	else // speed_motor3 is the biggest
	{
		if( fabs(speed_motor3 * 12.85) > 63.0 )
			ratio = fabs(speed_motor3 * 12.85) / 63.0;
	}
}
else
{
	if( fabs(speed_motor2 * 12.85) > fabs(speed_motor3 * 12.85) )
        { // speed_motor2 is the biggest
		if( fabs(speed_motor2 * 12.85) > 63.0 )
			ratio = fabs(speed_motor2 * 12.85) / 63.0;

        }
        else // speed_motor3 is the biggest
        {
		if( fabs(speed_motor3 * 12.85) > 63.0 )
			ratio = fabs(speed_motor3 * 12.85) / 63.0;

        }
}

  write_RoboClaw_drive_M1(128, int32_t( 64 + (-speed_motor1/ratio * 12.85) ));
  write_RoboClaw_drive_M2(128, int32_t( 64 + (-speed_motor3/ratio * 12.85) ));
  write_RoboClaw_drive_M1(129, int32_t( 64 + (-speed_motor2/ratio * 12.85) ));


}

void DriveRoboClaw::compute_motor_speed(double x, double y, double z)
{

	speed_motor1 = (-sin(theta1) * x + cos(theta1) * y + R * z) / RAYON;
	speed_motor2 = (-sin(theta2) * x + cos(theta2) * y + R * z) / RAYON;
	speed_motor3 = (-sin(theta3) * x + cos(theta3) * y + R * z) / RAYON;

        printf("speed_motor1 : %f speed_motor2 : %f speed_motor3 : %f \n", speed_motor1, speed_motor2, speed_motor3);

}

/***********************/
/* ROBO CLAW FUNCTIONS */
/***********************/

/* Without ENCODERS */

// Used to change the speed value of motor 1
void DriveRoboClaw::write_RoboClaw_forward_M1(char addr, int32_t speed)
{
    char checkSUM;
    checkSUM =
        (addr + 0 + ((char) (speed & 0xFF))) & 0x7F;

        unsigned char commands[4];
        commands[0] = addr;
        commands[1] = 0;
        commands[2] = ((char) (speed & 0xFF));
        commands[3] = checkSUM;

        write(fd, commands, 4);  //Send data
}

// Used to change the speed value of motor 1
void DriveRoboClaw::write_RoboClaw_backward_M1(char addr, int32_t speed)
{
    char checkSUM;
    checkSUM =
        (addr + 1 + ((char) (speed & 0xFF))) & 0x7F;

        unsigned char commands[4];
        commands[0] = addr;
        commands[1] = 1;
        commands[2] = ((char) (speed & 0xFF));
        commands[3] = checkSUM;

        write(fd, commands, 4);  //Send data
}

// Used to change the speed value of motor 2
void DriveRoboClaw::write_RoboClaw_forward_M2(char addr, int32_t speed)
{
    char checkSUM;
    checkSUM =
        (addr + 4 + ((char) (speed & 0xFF))) & 0x7F;

        unsigned char commands[4];
        commands[0] = addr;
        commands[1] = 4;
        commands[2] = ((char) (speed & 0xFF));
        commands[3] = checkSUM;

        write(fd, commands, 4);  //Send data
}

// Used to change the speed value of motor 2
void DriveRoboClaw::write_RoboClaw_backward_M2(char addr, int32_t speed)
{
    char checkSUM;
    checkSUM =
        (addr + 5 + ((char) (speed & 0xFF))) & 0x7F;

        unsigned char commands[4];
        commands[0] = addr;
        commands[1] = 5;
        commands[2] = ((char) (speed & 0xFF));
        commands[3] = checkSUM;

        write(fd, commands, 4);  //Send data
}

// Used to change the speed value of motor 2
void DriveRoboClaw::write_RoboClaw_drive_M1(char addr, int32_t speed)
{
    char checkSUM;
    checkSUM =
        (addr + 6 + ((char) (speed & 0xFF))) & 0x7F;

        unsigned char commands[4];
        commands[0] = addr;
        commands[1] = 6;
        commands[2] = ((char) (speed & 0xFF));
        commands[3] = checkSUM;

        write(fd, commands, 4);  //Send data
}

// Used to change the speed value of motor 2
void DriveRoboClaw::write_RoboClaw_drive_M2(char addr, int32_t speed)
{
    char checkSUM;
    checkSUM =
        (addr + 7 + ((char) (speed & 0xFF))) & 0x7F;

        unsigned char commands[4];
        commands[0] = addr;
        commands[1] = 7;
        commands[2] = ((char) (speed & 0xFF));
        commands[3] = checkSUM;

        write(fd, commands, 4);  //Send data
}



/* With ENCODERS */

// Used to change the speed value of motor 1
void DriveRoboClaw::write_RoboClaw_speed_M1(char addr, int32_t speed)
{
    char checkSUM;
    checkSUM =
        (addr + 35 + ((char) ((speed >> 24) & 0xFF)) +
         ((char) ((speed >> 16) & 0xFF)) + ((char) ((speed >> 8) & 0xFF)) +
         ((char) (speed & 0xFF))) & 0x7F;

        unsigned char commands[7];
        commands[0] = addr;
        commands[1] = 35;
        commands[2] = ((char) ((speed >> 24) & 0xFF));
        commands[3] = ((char) ((speed >> 16) & 0xFF));
        commands[4] = ((char) ((speed >> 8) & 0xFF));
        commands[5] = ((char) (speed & 0xFF));
        commands[6] = checkSUM;

        write(fd, commands, 7);  //Send data
}

// Used to change the speed value of motor 2
void DriveRoboClaw::write_RoboClaw_speed_M2(char addr, int32_t speed)
{
    char checkSUM;
    checkSUM =
        (addr + 36 + ((char) ((speed >> 24) & 0xFF)) +
         ((char) ((speed >> 16) & 0xFF)) + ((char) ((speed >> 8) & 0xFF)) +
         ((char) (speed & 0xFF))) & 0x7F;

        unsigned char commands[7];
        commands[0] = addr;
        commands[1] = 36;
        commands[2] = ((char) ((speed >> 24) & 0xFF));
        commands[3] = ((char) ((speed >> 16) & 0xFF));
        commands[4] = ((char) ((speed >> 8) & 0xFF));
        commands[5] = ((char) (speed & 0xFF));
        commands[6] = checkSUM;

	write(fd, commands, 7);  //Send data
}

// Used to change the speed value of motors 1 and 2
void DriveRoboClaw::write_RoboClaw_speed_M1M2(char addr, int32_t speedM1, int32_t speedM2)
{
    char checkSUM;

    checkSUM =
        (addr + 37 + ((char) ((speedM1 >> 24) & 0xFF)) +
         ((char) ((speedM1 >> 16) & 0xFF)) +
         ((char) ((speedM1 >> 8) & 0xFF)) + ((char) (speedM1 & 0xFF)) +
         ((char) ((speedM2 >> 24) & 0xFF)) +
         ((char) ((speedM2 >> 16) & 0xFF)) +
         ((char) ((speedM2 >> 8) & 0xFF)) +
         ((char) (speedM2 & 0xFF))) & 0x7F;

        unsigned char commands[11];
        commands[0] = addr;
        commands[1] = 37;
        commands[2] = ((char) ((speedM1 >> 24) & 0xFF));
        commands[3] = ((char) ((speedM1 >> 16) & 0xFF));
        commands[4] = ((char) ((speedM1 >> 8) & 0xFF));
        commands[5] = ((char) (speedM1 & 0xFF));
        commands[6] = ((char) ((speedM2 >> 24) & 0xFF));
        commands[7] = ((char) ((speedM2 >> 16) & 0xFF));
        commands[8] = ((char) ((speedM2 >> 8) & 0xFF));
        commands[9] = ((char) (speedM2 & 0xFF));
        commands[10] = checkSUM;

        write(fd, commands, 11);  //Send data
}

// Used to change the speed value of motor 1 and 2 during a specific distance
void DriveRoboClaw::write_RoboClaw_speed_dist_M1M2(char addr, int32_t speedM1,
                                    int32_t distanceM1,
                                    int32_t speedM2,
                                    int32_t distanceM2)
{
    char checkSUM;
    checkSUM =
        (addr + 43 + ((char) ((speedM1 >> 24) & 0xFF)) +
         ((char) ((speedM1 >> 16) & 0xFF)) +
         ((char) ((speedM1 >> 8) & 0xFF)) + ((char) (speedM1 & 0xFF)) +
         ((char) ((speedM2 >> 24) & 0xFF)) +
         ((char) ((speedM2 >> 16) & 0xFF)) +
         ((char) ((speedM2 >> 8) & 0xFF)) + ((char) (speedM2 & 0xFF)) +
         ((char) ((distanceM1 >> 24) & 0xFF)) +
         ((char) ((distanceM1 >> 16) & 0xFF)) +
         ((char) ((distanceM1 >> 8) & 0xFF)) +
         ((char) (distanceM1 & 0xFF)) +
         ((char) ((distanceM2 >> 24) & 0xFF)) +
         ((char) ((distanceM2 >> 16) & 0xFF)) +
         ((char) ((distanceM2 >> 8) & 0xFF)) +
         ((char) (distanceM2 & 0xFF)) + 1) & 0x7F;

        unsigned char commands[20];
        commands[0] = addr;
        commands[1] = 43;
        commands[2] = ((char) ((speedM1 >> 24) & 0xFF));
        commands[3] = ((char) ((speedM1 >> 16) & 0xFF));
        commands[4] = ((char) ((speedM1 >> 8) & 0xFF));
        commands[5] = ((char) (speedM1 & 0xFF));
        commands[6] = ((char) ((distanceM1 >> 24) & 0xFF));
        commands[7] = ((char) ((distanceM1 >> 16) & 0xFF));
        commands[8] = ((char) ((distanceM1 >> 8) & 0xFF));
        commands[9] = ((char) (distanceM1 & 0xFF));
        commands[10] = ((char) ((speedM2 >> 24) & 0xFF));
        commands[11] = ((char) ((speedM2 >> 16) & 0xFF));
        commands[12] = ((char) ((speedM2 >> 8) & 0xFF));
        commands[13] = ((char) (speedM2 & 0xFF));
        commands[14] = ((char) ((distanceM2 >> 24) & 0xFF));
        commands[15] = ((char) ((distanceM2 >> 16) & 0xFF));
        commands[16] = ((char) ((distanceM2 >> 8) & 0xFF));
        commands[17] = ((char) (distanceM2 & 0xFF));
        commands[18] = 1;
        commands[19] = checkSUM;

        write(fd, commands, 20);  //Send data
}


void DriveRoboClaw::rotate(int32_t speed)
{

    char checkSUM;

  checkSUM =
        (128 + 35 + ((char) ((speed >> 24) & 0xFF)) +
         ((char) ((speed >> 16) & 0xFF)) + ((char) ((speed >> 8) & 0xFF)) +
         ((char) (speed & 0xFF))) & 0x7F;

	unsigned char commands[7];
	commands[0] = 128;
	commands[1] = 35;
	commands[2] = ((char) ((speed >> 24) & 0xFF));
	commands[3] = ((char) ((speed >> 16) & 0xFF));
	commands[4] = ((char) ((speed >> 8) & 0xFF));
	commands[5] = ((char) (speed & 0xFF));
	commands[6] = checkSUM;

	write(fd, commands, 7);  //Send data

}


int main(int argc, char **argv)
{


    ros::init(argc, argv, "RoboClaw_Driver");
    DriveRoboClaw drive;
    // Refresh rate
    ros::Rate loop_rate(60);                                // 35 with bluetooth


    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
/*
	drive.write_RoboClaw_drive_M1(128, 100);
	ros::Duration(1.0).sleep();
	drive.write_RoboClaw_drive_M1(128, 50);
	ros::Duration(1.0).sleep();
	drive.write_RoboClaw_drive_M1(128, 64);
	drive.write_RoboClaw_drive_M2(128, 50);
        ros::Duration(1.0).sleep();
	drive.write_RoboClaw_drive_M1(129, 50);
	drive.write_RoboClaw_drive_M2(128, 64);
        ros::Duration(1.0).sleep();
	drive.write_RoboClaw_drive_M1(129, 64);
        ros::Duration(1.0).sleep();
*/
/*
	drive.write_RoboClaw_speed_M1M2(128, 10000, 0);
	ros::Duration(1.0).sleep();
	drive.write_RoboClaw_speed_M1M2(128, 0, 10000);
	ros::Duration(1.0).sleep();
	drive.write_RoboClaw_speed_M1M2(128, 0, 0);
	drive.write_RoboClaw_speed_M1M2(129, 10000, 0);
	ros::Duration(1.0).sleep();
	drive.write_RoboClaw_speed_M1M2(129, 0, 0);
	ros::Duration(1.0).sleep();

        drive.write_RoboClaw_speed_M1M2(128, -30000, 0);
        ros::Duration(1.0).sleep();
        drive.write_RoboClaw_speed_M1M2(128, 0, -30000);
        ros::Duration(1.0).sleep();
        drive.write_RoboClaw_speed_M1M2(128, 0, 0);
        drive.write_RoboClaw_speed_M1M2(129, -30000, 0);
        ros::Duration(1.0).sleep();
        drive.write_RoboClaw_speed_M1M2(129, 0, 0);
        ros::Duration(1.0).sleep();
*/
/*
        drive.write_RoboClaw_speed_M1(128, 5000);
        ros::Duration(1.0).sleep();
	drive.write_RoboClaw_speed_M1(128, 10000);
    	ros::Duration(1.0).sleep();
        drive.write_RoboClaw_speed_M1(128, 20000);
    	ros::Duration(1.0).sleep();
        drive.write_RoboClaw_speed_M1(128, 30000);
    	ros::Duration(1.0).sleep();
        drive.write_RoboClaw_speed_M1(128, 40000);
    	ros::Duration(1.0).sleep();
        drive.write_RoboClaw_speed_M1(128, 50000);
    	ros::Duration(1.0).sleep();
        drive.write_RoboClaw_speed_M1(128, 0);
    	ros::Duration(1.0).sleep();
*/
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}

