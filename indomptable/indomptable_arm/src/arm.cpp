#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

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
// For I2C : 
#include <linux/i2c-dev.h>
// For gettimeofday
#include <getopt.h>
#include <sys/time.h>


#define False 0
#define True  1 
#define FALSE 0
#define TRUE  1

#define SLEEP_COEFF 1500 //2000

#define SSCDEVICE "/dev/ttyS0"
#define BAUDRATE B115200

#define CoxaLength 43      //Length of the Coxa [mm]
#define FemurLength 76      //Length of the Femur [mm]
#define TibiaLength 66     //NEW Lenght of the Tibia [mm]
//#define CoxaAngle 0      //Default Coxa setup angle

// SERVO OFFSET
#define SERVO_OFFSET0   (-88)
#define SERVO_OFFSET4   80
#define SERVO_OFFSET8   194
#define SERVO_OFFSET12   101

#define STOCK_HEIGHT	80

class indomptableARM {
  public:
    indomptableARM();

    ros::Subscriber pose_sub_;
    ros::Subscriber pump_sub_;
    ros::Publisher coxa_pub;
    ros::Publisher femur_pub;
    ros::Publisher tibia_pub;
    ros::Publisher ankle_pub;
    ros::Publisher roll_pub;
    ros::Publisher hand_pub;
    ros::Publisher pump_pub;

  private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
    void pumpCallback(const std_msgs::Int32::ConstPtr & pumpfeedback);
    void LegIK(signed int IKFeetPosX, signed int IKFeetPosY, signed int IKFeetPosZ);
    void ServoDriver();
    void FreeServos();
    void takeCDinTotem(signed int height);
    void takeBARinTotem(void);
    void takeGround(signed int x, signed int y);

    ros::NodeHandle nh;

    int input_y;
    std::string input_name;
    std::string default_param;

    std_msgs::Int32 des_pump;
    int pump_ok;




//====================================================================
//[ANGLES]
double CoxaAngle;   //Actual Angle of the Right Front Leg
double FemurAngle;
double TibiaAngle;
double AnkleAngle;
double RollAngle;
double HandAngle;
//--------------------------------------------------------------------
//[POSITIONS]
signed int RFPosX;      //Actual Position of the Right Front Leg
signed int RFPosY;
signed int RFPosZ;
//--------------------------------------------------------------------
//[VARIABLES]
int ActualGaitSpeed;

double DesAnkleAngle;
double IKFemurAngle;       //Output Angle of Femur in degrees
double IKTibiaAngle;       //Output Angle of Tibia in degrees
double IKCoxaAngle;        //Output Angle of Coxa in degrees

/*
// Serial
int ser_fd_ssc;
struct termios oldtio_ssc, newtio_ssc;
*/

};

indomptableARM::indomptableARM()
{

    nh.param<std::string>("arm_pose_name", input_name, "left");
    nh.param("arm_pose_y", input_y, 0);
    //printf("%s\n", input_name.c_str());
    //printf("%i\n", input_y);
    std::string tmp_string = input_name;
    //tmp_string.append(input_name);
    tmp_string.append("_arm_pose");
    pose_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("arm_pose", 5, &indomptableARM::poseCallback, this);

    tmp_string = input_name;
    tmp_string.append("_soulder_roll_joint");
    coxa_pub = nh.advertise < std_msgs::Float64 > ("soulder_roll_joint", 5);
    tmp_string = input_name;
    tmp_string.append("_soulder_lift_joint");
    femur_pub = nh.advertise < std_msgs::Float64 > ("soulder_lift_joint", 5);
    tmp_string = input_name;
    tmp_string.append("_elbow_joint");
    tibia_pub = nh.advertise < std_msgs::Float64 > ("elbow_joint", 5);
    tmp_string = input_name;
    tmp_string.append("_wrist_joint");
    ankle_pub = nh.advertise < std_msgs::Float64 > ("wrist_joint", 5);
    tmp_string = input_name;
    tmp_string.append("_hand_joint");
    roll_pub = nh.advertise < std_msgs::Float64 > ("hand_joint", 5);
    tmp_string = input_name;
    tmp_string.append("_hand");
    hand_pub = nh.advertise < std_msgs::Float64 > ("hand", 5);

    tmp_string = input_name;
    tmp_string.append("_pump");
    pump_pub = nh.advertise < std_msgs::Int32 > ("pump", 5);
    tmp_string = input_name;
    tmp_string.append("_pump_feedback");
    pump_sub_ = nh.subscribe < std_msgs::Int32 > ("pump_feedback", 5, &indomptableARM::pumpCallback, this);

//--------------------------------------------------------------------
//[POSITIONS]
RFPosX = 0;      //Actual Position of the Right Front Leg
RFPosY = 0;
RFPosZ = 0;
//--------------------------------------------------------------------
//[VARIABLES]

IKFemurAngle = 0;       //Output Angle of Femur in degrees
IKTibiaAngle= 0;       //Output Angle of Tibia in degrees
IKCoxaAngle = 0;        //Output Angle of Coxa in degrees

ActualGaitSpeed = 200;

DesAnkleAngle = 0;
RollAngle = 0;
HandAngle = 0;

    des_pump.data = 0;
    pump_ok = 0;

/*
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
*/
}

void indomptableARM::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose)
{
	if( ((int)(pose->pose.position.y * 1000)) != 0 ) {
		takeGround((int)(pose->pose.position.x * 1000), (int)(pose->pose.position.y * 1000));
	}
	else {
		if( ((int)(pose->pose.position.z * 1000)) == 0 ) {
			takeBARinTotem();
		}
		else {
	 		takeCDinTotem((int)(pose->pose.position.z * 1000));
		}
	}
}

void indomptableARM::pumpCallback(const std_msgs::Int32::ConstPtr & pumpfeedback)
{
    if(pumpfeedback->data == des_pump.data)
	pump_ok = 1;
    else 
    	pump_ok = 0;

}

/****** Inverse Kinematics functions *******/

void indomptableARM::LegIK(signed int IKFeetPosX, signed int IKFeetPosY, signed int IKFeetPosZ){

//--------------------------------------------------------------------
//[VARIABLES]

//Leg Inverse Kinematics
double IKFeetPosXZ;        //Length between the coxa and feet
double IKSW;             //Length between shoulder and wrist
double IKA1;             //Angle between SW line and the ground in rad
double IKA2;             //?
char IKSolution;         //Output true if the solution is possible
char IKSolutionWarning;      //Output true if the solution is NEARLY possible
char IKSolutionError;      //Output true if the solution is NOT possible


        //Length between the Coxa and Feet
        IKFeetPosXZ =  sqrt((double)((IKFeetPosX*IKFeetPosX)+(IKFeetPosZ*IKFeetPosZ)));

        //IKSW - Length between shoulder and wrist
        IKSW = sqrt((double)(((IKFeetPosXZ-CoxaLength)*(IKFeetPosXZ-CoxaLength))+(IKFeetPosY*IKFeetPosY)));
        //IKSW2 = sqrt((double)(((IKFeetPosXZ-CoxaLength)*(IKFeetPosXZ-CoxaLength))+(IKFeetPosY*IKFeetPosY)));
	
	//printf("%i %f %f\n", IKFeetPosX, IKFeetPosXZ, IKSW);
        
	//IKA1 - Angle between SW line and the ground in rad
        IKA1 = atan2(IKFeetPosXZ-CoxaLength, IKFeetPosY);

        //IKA2 - ?
        IKA2 = acos( ((double)((FemurLength*FemurLength) - (TibiaLength*TibiaLength)) + (IKSW*IKSW)) / ((double)(2*FemurLength) * IKSW) );

        //IKFemurAngle
        //IKFemurAngle = ( -(IKA1 + IKA2) * 180.0 / 3.141592 )+90;
        IKFemurAngle = ( -(IKA1 + IKA2) )+1.570796;
	//printf("%f %f %f %f\n", (IKA1 + IKA2), -(IKA1 + IKA2), ( -(IKA1 + IKA2) * 180.0 / 3.141592 ), ( -(IKA1 + IKA2) * 180.0 / 3.141592 )+90);
	//printf("%f %f\n", IKFemurAngle, ( -(IKA1 + IKA2) )+1.570796);

        //IKTibiaAngle
        //IKTibiaAngle = -( 90-(acos( ( (double)(FemurLength*FemurLength) + (TibiaLength*TibiaLength) - (IKSW*IKSW) ) / ((double)(2*FemurLength*TibiaLength)) )*180.0 / 3.141592) );
        IKTibiaAngle = -( 1.570796-(acos( ( (double)(FemurLength*FemurLength) + (TibiaLength*TibiaLength) - (IKSW*IKSW) ) / ((double)(2*FemurLength*TibiaLength)) )) );
	//printf("%f %f\n", IKTibiaAngle, -( 1.570796-(acos( ( (double)(FemurLength*FemurLength) + (TibiaLength*TibiaLength) - (IKSW*IKSW) ) / ((double)(2*FemurLength*TibiaLength)) )) ));

        //IKCoxaAngle
        //GetBoogTan(IKFeetPosZ, IKFeetPosX);
        //IKCoxaAngle = ((atan2(IKFeetPosZ, IKFeetPosX)*180.0) / 3.141592);
        IKCoxaAngle = ((atan2(IKFeetPosZ, IKFeetPosX)));
	//printf("%f %f\n", IKCoxaAngle, ((atan2(IKFeetPosZ, IKFeetPosX))));

        //Set the Solution quality   
        if(IKSW < (double)(FemurLength+TibiaLength-30)) {
                IKSolution = TRUE;
        }
        else {
                if(IKSW < (double)(FemurLength+TibiaLength)) {
                        IKSolutionWarning = TRUE;
                }
                else {
                        IKSolutionError = TRUE;
                }
        }



        return;
}


void indomptableARM::takeCDinTotem(signed int height){

        LegIK((int)(90), (int)(height), (int)(0));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
	RollAngle = 1.570796;
	ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(130), (int)(height), (int)(0));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed)*SLEEP_COEFF);

        LegIK((int)(150), (int)(height), (int)(0));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
	RollAngle = 1.570796;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

	// GRIP
	HandAngle = 1.0;
        ServoDriver();

	usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(80), (int)(height), (int)(0));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);


        LegIK((int)(80), (int)(height), (int)(25));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(100), (int)(STOCK_HEIGHT+20), (int)(25));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 0;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(130), (int)(STOCK_HEIGHT+20), (int)(0));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);




        LegIK((int)(130), (int)(STOCK_HEIGHT), (int)(0));
        DesAnkleAngle = 1.570796*2;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);
/*
        LegIK((int)(0), (int)(STOCK_HEIGHT), (int)(0));
        DesAnkleAngle = 1.570796*2;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);
*/
	// RELEASE
	HandAngle = -0.5;
        ServoDriver();

	usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(130), (int)(STOCK_HEIGHT), (int)(0));
        DesAnkleAngle = 1.570796*2;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 0;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(110), (int)(0), (int)(0));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 0;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);


}

void indomptableARM::takeBARinTotem(void){

        LegIK((int)(150), (int)(0), (int)(50));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(150), (int)(0), (int)(0));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        // PUMP

	usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(110), (int)(0), (int)(0));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 500;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(120), (int)(STOCK_HEIGHT), (int)(0));
        DesAnkleAngle = 1.570796*2;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 500;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);
/*
        LegIK((int)(0), (int)(STOCK_HEIGHT), (int)(0));
        DesAnkleAngle = 1.570796*2;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 500;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);
*/
	// UNPUMP

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(120), (int)(STOCK_HEIGHT), (int)(0));
        DesAnkleAngle = 1.570796*2;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 1.570796;
        ActualGaitSpeed = 500;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(110), (int)(0), (int)(0));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 0;
	ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);


}

void indomptableARM::takeGround(signed int x, signed int y){

        LegIK((int)(x), (int)(-90), (int)(y));
        DesAnkleAngle = -1.570796;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = -IKFemurAngle;
        TibiaAngle = (1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = -IKCoxaAngle;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        // pump
        pump_ok = 0;
        des_pump.data = 1;
        pump_pub.publish(des_pump);
        usleep(50000);
        ros::spinOnce();
        usleep(50000);
        ros::spinOnce();
        while(pump_ok == 0) {
                pump_pub.publish(des_pump);
                usleep(50000);
                ros::spinOnce();
                usleep(50000);
                ros::spinOnce();
        }

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);



        LegIK((int)(x), (int)(-10), (int)(y));
        DesAnkleAngle = -1.570796;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = -IKFemurAngle;
        TibiaAngle = (1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = -IKCoxaAngle;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();
        
        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);
        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        LegIK((int)(x), (int)(-90), (int)(y));
        DesAnkleAngle = -1.570796;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = -IKFemurAngle;
        TibiaAngle = (1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = -IKCoxaAngle;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        // unpump
        pump_ok = 0;
        des_pump.data = 0;
        pump_pub.publish(des_pump);
        usleep(50000);
        ros::spinOnce();
        usleep(50000);
        ros::spinOnce();
        while(pump_ok == 0) {
                pump_pub.publish(des_pump);
                usleep(50000);
                ros::spinOnce();
                usleep(50000);
                ros::spinOnce();
        }

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

        DesAnkleAngle = 0;
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);


        LegIK((int)(110), (int)(0), (int)(0));
        DesAnkleAngle = 0;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = IKFemurAngle;
        TibiaAngle = -(1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = 0;
        ActualGaitSpeed = 300;
        printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
        ServoDriver();

        usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

}




//--------------------------------------------------------------------
//  ;[SERVO DRIVER] Updates the positions of the servos    
void indomptableARM::ServoDriver(void){
/*
        char Serout[260]={0};
        int temp = 0;


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
 	std_msgs::Float64 tmp;

	tmp.data = CoxaAngle;
	coxa_pub.publish(tmp);
	tmp.data = FemurAngle;
	femur_pub.publish(tmp);
	tmp.data = TibiaAngle;
	tibia_pub.publish(tmp);
	tmp.data = AnkleAngle;
	ankle_pub.publish(tmp);
	tmp.data = RollAngle;
	roll_pub.publish(tmp);
	tmp.data = HandAngle;
	hand_pub.publish(tmp);

        return;
}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
void indomptableARM::FreeServos()
{
/*
        int x =0;
        char Serout[260]={0};
        for ( x=0; x <= 31; x++ )
        {
                sprintf(Serout, "%s#%dP0 ", Serout, x);
        }
        sprintf(Serout, "%sT200\r", Serout);

        // write to serial if connected
        if ( ser_fd_ssc )
                write(ser_fd_ssc, &Serout, sizeof(Serout));
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
    ros::init(argc, argv, "indomptable_arm");
    indomptableARM indomptablearm;
    // Refresh rate
    ros::Rate loop_rate(5);                                // 35 with bluetooth
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}

