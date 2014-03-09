#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>


#include <cstring>
#include <iostream>
#include <cstdio>

#include <sstream>
#include <math.h>
#include <vector>

#ifdef USE_BOOST
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/array.hpp>
#else
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

#endif

#define theta1 (3.14159265359 - 2.09439510239)
#define theta2 (3.14159265359)
#define theta3 (3.14159265359 + 2.09439510239)
#define R (0.09)
#define RAYON (0.03)



class DriveRoboClaw {
	public:
		DriveRoboClaw();
		void rotate(int32_t speed);

		void write_RoboClaw_forward_M1(char addr, int32_t speed);
		void write_RoboClaw_backward_M1(char addr, int32_t speed);
		void write_RoboClaw_forward_M2(char addr, int32_t speed);
		void write_RoboClaw_backward_M2(char addr, int32_t speed);

		void write_RoboClaw_PID_M1(char addr, int32_t D, int32_t P, int32_t I, int32_t QQPS);
		void write_RoboClaw_PID_M2(char addr, int32_t D, int32_t P, int32_t I, int32_t QQPS);
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

#ifdef USE_BOOST
            	std::string port_; ///< @brief The serial port the driver is attached to
            	uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
            			
		boost::shared_ptr<boost::asio::serial_port> serial_;

    		boost::asio::io_service io;
    		boost::array < uint8_t, 16 > raw_bytes_;
    		boost::array < uint8_t, 2 > speed_;
#else
                int fd; // file description for the serial port
                struct termios port_settings;      // structure to store the port settings in
#endif

		double speed_motor1;
		double speed_motor2;
		double speed_motor3;

};

DriveRoboClaw::DriveRoboClaw()
{

#ifdef USE_BOOST
	port_="/dev/ttyROBOCLAW";
	baud_rate_=115200;

    	try {
        	//boost::asio::serial_port serial_(io, port_);
		serial_ = boost::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(io, port_));
        	//serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
		//boost::asio::write(serial_,boost::asio::buffer(sendThis,1));
		serial_->set_option(boost::asio::serial_port_base::character_size(8));
		serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		serial_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
	}
    	catch(boost::system::system_error ex) {
        	ROS_ERROR("Error instantiating laser object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
        	//return -1;
    	}
#else

	fd = open("/dev/ttyROBOCLAW", O_RDWR | O_NOCTTY | O_NDELAY);

	if(fd == -1) // if open is unsucessful
	{
		//perror("open_port: Unable to open /dev/ttyS0 - ");
		printf("open_port: Unable to open /dev/ttyROBOCLAW. \n");
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
#endif

	speed_motor1 = 0;
	speed_motor2 = 0;
	speed_motor3 = 0;

	vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 5, &DriveRoboClaw::velCallback, this);

	//write_RoboClaw_PID_M1(128, 16384, 65536, 32768, 44000);
	//write_RoboClaw_PID_M2(128, 0x00004000, 0x00010000, 0x00008000, 44000);
	write_RoboClaw_PID_M1(128, 70384, 170536, 100768, 120000); // 94000
	write_RoboClaw_PID_M2(128, 70384, 170536, 100768, 120000);

	write_RoboClaw_PID_M1(129, 70384, 170536, 100768, 120000);

}


void DriveRoboClaw::velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
	compute_motor_speed(vel->linear.x, -vel->linear.y, -vel->angular.z);
	/*
	   write_RoboClaw_speed_M1M2(128, int32_t(speed_motor1 * 20000), int32_t(speed_motor2 * 20000));
	   write_RoboClaw_speed_M1(129, int32_t(speed_motor3 * 20000));  
	 */

	// TODO : brider la vitesse des moteurs
	// TODO : SI un moteur va trop vite, prendre son ratio par rapport a vitesse max et l'appliquer aux autres moteurs

	double ratio = 1.0;
	/*
	   if( fabs(speed_motor1) > fabs(speed_motor2) )
	   {
	   if( fabs(speed_motor1) > fabs(speed_motor3) )
	   { // speed_motor1 is the biggest
	   if( fabs(speed_motor1) > 63.0 )
	   ratio = fabs(speed_motor1) / 63.0;
	   }
	   else // speed_motor3 is the biggest
	   {
	   if( fabs(speed_motor3) > 63.0 )
	   ratio = fabs(speed_motor3) / 63.0;
	   }
	   }
	   else
	   {
	   if( fabs(speed_motor2) > fabs(speed_motor3) )
	   { // speed_motor2 is the biggest
	   if( fabs(speed_motor2) > 63.0 )
	   ratio = fabs(speed_motor2) / 63.0;

	   }
	   else // speed_motor3 is the biggest
	   {
	   if( fabs(speed_motor3) > 63.0 )
	   ratio = fabs(speed_motor3) / 63.0;

	   }
	   }


	   write_RoboClaw_drive_M1(128, int32_t( 64 + (-speed_motor1/ratio * 12.85) ));
	   write_RoboClaw_drive_M2(128, int32_t( 64 + (-speed_motor3/ratio * 12.85) ));
	   write_RoboClaw_drive_M1(129, int32_t( 64 + (-speed_motor2/ratio * 12.85) ));
	 */

	write_RoboClaw_speed_M1M2(128, int32_t(speed_motor1 * 5000.0), -int32_t(speed_motor2 * 5000.0));
	write_RoboClaw_speed_M1(129, int32_t(speed_motor3 * 5000.0));  


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
#ifdef USE_BOOST
	serial_->write_some(boost::asio::buffer(commands, 4));
	//boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
	write(fd, commands, 4);  //Send data
#endif
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

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 4));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 4);  //Send data
#endif

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

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 4));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 4);  //Send data
#endif

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

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 4));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 4);  //Send data
#endif

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

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 4));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 4);  //Send data
#endif

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

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 4));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 4);  //Send data
#endif

}



/* With ENCODERS */

// of motor 1
void DriveRoboClaw::write_RoboClaw_PID_M1(char addr, int32_t D, int32_t P, int32_t I, int32_t QQPS)
{
	char checkSUM;
	checkSUM =
		(addr + 28 + ((char) ((D >> 24) & 0xFF)) +
		 ((char) ((D >> 16) & 0xFF)) + ((char) ((D >> 8) & 0xFF)) +
		 ((char) (D & 0xFF)) + ((char) ((P >> 24) & 0xFF)) +
		 ((char) ((P >> 16) & 0xFF)) + ((char) ((P >> 8) & 0xFF)) +
		 ((char) (P & 0xFF)) + ((char) ((I >> 24) & 0xFF)) +
		 ((char) ((I >> 16) & 0xFF)) + ((char) ((I >> 8) & 0xFF)) +
		 ((char) (I & 0xFF)) + ((char) ((QQPS >> 24) & 0xFF)) +
		 ((char) ((QQPS >> 16) & 0xFF)) + ((char) ((QQPS >> 8) & 0xFF)) +
		 ((char) (QQPS & 0xFF))) & 0x7F;

	unsigned char commands[19];
	commands[0] = addr;
	commands[1] = 28;
	commands[2] = ((char) ((D >> 24) & 0xFF));
	commands[3] = ((char) ((D >> 16) & 0xFF));
	commands[4] = ((char) ((D >> 8) & 0xFF));
	commands[5] = ((char) (D & 0xFF));
	commands[6] = ((char) ((P >> 24) & 0xFF));
	commands[7] = ((char) ((P >> 16) & 0xFF));
	commands[8] = ((char) ((P >> 8) & 0xFF));
	commands[9] = ((char) (P & 0xFF));
	commands[10] = ((char) ((I >> 24) & 0xFF));
	commands[11] = ((char) ((I >> 16) & 0xFF));
	commands[12] = ((char) ((I >> 8) & 0xFF));
	commands[13] = ((char) (I & 0xFF));
	commands[14] = ((char) ((QQPS >> 24) & 0xFF));
	commands[15] = ((char) ((QQPS >> 16) & 0xFF));
	commands[16] = ((char) ((QQPS >> 8) & 0xFF));
	commands[17] = ((char) (QQPS & 0xFF));
	commands[18] = checkSUM;

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 19));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 19);  //Send data
#endif

}

// of motor 2
void DriveRoboClaw::write_RoboClaw_PID_M2(char addr, int32_t D, int32_t P, int32_t I, int32_t QQPS)
{
	char checkSUM;
	checkSUM =
		(addr + 29 + ((char) ((D >> 24) & 0xFF)) +
		 ((char) ((D >> 16) & 0xFF)) + ((char) ((D >> 8) & 0xFF)) +
		 ((char) (D & 0xFF)) + ((char) ((P >> 24) & 0xFF)) +
		 ((char) ((P >> 16) & 0xFF)) + ((char) ((P >> 8) & 0xFF)) +
		 ((char) (P & 0xFF)) + ((char) ((I >> 24) & 0xFF)) +
		 ((char) ((I >> 16) & 0xFF)) + ((char) ((I >> 8) & 0xFF)) +
		 ((char) (I & 0xFF)) + ((char) ((QQPS >> 24) & 0xFF)) +
		 ((char) ((QQPS >> 16) & 0xFF)) + ((char) ((QQPS >> 8) & 0xFF)) +
		 ((char) (QQPS & 0xFF))) & 0x7F;

	unsigned char commands[19];
	commands[0] = addr;
	commands[1] = 29;
	commands[2] = ((char) ((D >> 24) & 0xFF));
	commands[3] = ((char) ((D >> 16) & 0xFF));
	commands[4] = ((char) ((D >> 8) & 0xFF));
	commands[5] = ((char) (D & 0xFF));
	commands[6] = ((char) ((P >> 24) & 0xFF));
	commands[7] = ((char) ((P >> 16) & 0xFF));
	commands[8] = ((char) ((P >> 8) & 0xFF));
	commands[9] = ((char) (P & 0xFF));
	commands[10] = ((char) ((I >> 24) & 0xFF));
	commands[11] = ((char) ((I >> 16) & 0xFF));
	commands[12] = ((char) ((I >> 8) & 0xFF));
	commands[13] = ((char) (I & 0xFF));
	commands[14] = ((char) ((QQPS >> 24) & 0xFF));
	commands[15] = ((char) ((QQPS >> 16) & 0xFF));
	commands[16] = ((char) ((QQPS >> 8) & 0xFF));
	commands[17] = ((char) (QQPS & 0xFF));
	commands[18] = checkSUM;

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 19));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 19);  //Send data
#endif

}


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

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 7));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 7);  //Send data
#endif

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

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 7));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 7);  //Send data
#endif

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

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 11));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 11);  //Send data
#endif

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

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 20));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 20);  //Send data
#endif

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

#ifdef USE_BOOST
        serial_->write_some(boost::asio::buffer(commands, 7));
        //boost::asio::write(serial_,boost::asio::buffer(commands,4));
#else
        write(fd, commands, 7);  //Send data
#endif


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
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

