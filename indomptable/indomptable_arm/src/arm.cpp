#include "ros/ros.h"
#include "std_msgs/String.h"
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

#define SLEEP_COEFF 2000

#define SSCDEVICE "/dev/ttyUSB0"
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


class indomptableARM {
  public:
    indomptableARM();

    ros::Subscriber pose_sub_;

  private:
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
    void GetSinCos (float AngleDeg);
    void GetBoogTan(float BoogTanX, float BoogTanY);
    void LegIK(signed int IKFeetPosX, signed int IKFeetPosY, signed int IKFeetPosZ);


    ros::NodeHandle nh;

    int input_y;
    //string input_name;



//====================================================================
//[ANGLES]
float CoxaAngle;   //Actual Angle of the Right Front Leg
float FemurAngle;
float TibiaAngle;
float AnkleAngle;
//--------------------------------------------------------------------
//[POSITIONS]
signed int RFPosX;      //Actual Position of the Right Front Leg
signed int RFPosY;
signed int RFPosZ;
//--------------------------------------------------------------------
//[VARIABLES]
char Index;      //Index used for freeing the servos
char SSCDone;    //Char to check if SSC is done

float ABSAngleDeg;      //Absolute value of the Angle in Degrees
float AngleRad;     //Angle in Radian
float SinA;      //Output Sinus of the given Angle
float CosA;      //Output Cosinus of the given Angle

float BoogTan;      //Output BOOGTAN2(X/Y)
float SinB;             //Sin buffer for BodyRotX calculations
float CosB;             //Cos buffer for BodyRotX calculations
float SinG;             //Sin buffer for BodyRotZ calculations
float CosG;             //Cos buffer for BodyRotZ calculations
signed int TotalX;             //Total X distance between the center of the body and the feet
signed int TotalZ;             //Total Z distance between the center of the body and the feet
float DistCenterBodyFeet; //Total distance between the center of the body and the feet
float AngleCenterBodyFeetX; //Angle between the center of the body and the feet
signed int BodyIKPosX;         //Output Position X of feet with Rotation
signed int BodyIKPosY;         //Output Position Y of feet with Rotation
signed int BodyIKPosZ;         //Output Position Z of feet with Rotation

//Leg Inverse Kinematics
signed int IKFeetPosXZ;        //Length between the coxa and feet
float IKSW;             //Length between shoulder and wrist
float IKA1;             //Angle between SW line and the ground in rad
float IKA2;             //?
char IKSolution;         //Output true if the solution is possible
char IKSolutionWarning;      //Output true if the solution is NEARLY possible
char IKSolutionError;      //Output true if the solution is NOT possible
float IKFemurAngle;       //Output Angle of Femur in degrees
float IKTibiaAngle;       //Output Angle of Tibia in degrees
float IKCoxaAngle;        //Output Angle of Coxa in degrees

int ActualGaitSpeed;

signed int RFGaitPosX;   //Relative position corresponding to the Gait
signed int RFGaitPosY;
signed int RFGaitPosZ;
signed int RFGaitRotY;   //Relative rotation corresponding to the Gait

//Body position
signed int BodyPosX;      //Global Input for the position of the body
signed int BodyPosY;
signed int BodyPosZ;

signed int DesAnkleAngle;





};

indomptableARM::indomptableARM():
input_y(-60)//, input_name("left_to_take")
{

    nh.param("input_y", input_y, input_y);
    //nh.param("input_name", input_name, input_name);

    pose_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("arm_input", 5, &indomptableARM::poseCallback, this);

//--------------------------------------------------------------------
//[POSITIONS]
RFPosX = 0;      //Actual Position of the Right Front Leg
RFPosY = 0;
RFPosZ = 0;
//--------------------------------------------------------------------
//[VARIABLES]

ABSAngleDeg = 0;      //Absolute value of the Angle in Degrees
AngleRad = 0;     //Angle in Radian
SinA = 0;      //Output Sinus of the given Angle
CosA = 0;      //Output Cosinus of the given Angle

TotalX = 0;             //Total X distance between the center of the body and the feet
TotalZ = 0;             //Total Z distance between the center of the body and the feet
DistCenterBodyFeet = 0; //Total distance between the center of the body and the feet
AngleCenterBodyFeetX = 0; //Angle between the center of the body and the feet
BodyIKPosX = 0;         //Output Position X of feet with Rotation
BodyIKPosY = 0;         //Output Position Y of feet with Rotation
BodyIKPosZ = 0;         //Output Position Z of feet with Rotation

//Leg Inverse Kinematics
IKFeetPosXZ = 0;        //Length between the coxa and feet
IKSW = 0;             //Length between shoulder and wrist
IKA1 = 0;             //Angle between SW line and the ground in rad
IKA2 = 0;             //?
IKFemurAngle = 0;       //Output Angle of Femur in degrees
IKTibiaAngle= 0;       //Output Angle of Tibia in degrees
IKCoxaAngle = 0;        //Output Angle of Coxa in degrees

ActualGaitSpeed = 200;

RFGaitPosX = 0;   //Relative position corresponding to the Gait
RFGaitPosY = 0;
RFGaitPosZ = 0;
RFGaitRotY = 0;   //Relative rotation corresponding to the Gait

//Body position
BodyPosX = 0;      //Global Input for the position of the body
BodyPosY = 0;
BodyPosZ = 0;

DesAnkleAngle = 0;







}

void indomptableARM::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose)
{

	LegIK((int)(pose->pose.position.x * 1000), (int)(pose->pose.position.z * 1000), (int)(pose->pose.position.y * 1000));
	CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
	FemurAngle = IKFemurAngle;
	TibiaAngle = -(90 - IKTibiaAngle);
	AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
	printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);
	//ServoDriver();

	usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

}

/****** Inverse Kinematics functions *******/

/**--------------------------------------------------------------------
  [GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
  AngleDeg    - Input Angle in degrees
  SinA        - Output Sinus of AngleDeg
  CosA        - Output Cosinus of AngleDeg*/
void indomptableARM::GetSinCos (float AngleDeg) {

        //Get the absolute value of AngleDeg

        ABSAngleDeg = fabs(AngleDeg);



        //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
        if (AngleDeg < 0.0) {   //Negative values
                AngleDeg = 360.0-(ABSAngleDeg-((360*((int)(ABSAngleDeg/360.0)))));
        }
        else {            //Positive values
                AngleDeg = ABSAngleDeg-((360*((int)(ABSAngleDeg/360.0))));
        }

        if (AngleDeg < 180.0) {   //Angle between 0 and 180
                //Subtract 90 to shift range
                AngleDeg = AngleDeg -90.0;
                //Convert degree to radials
                AngleRad = (AngleDeg*3.141592)/180.0;

                CosA = -sin(AngleRad);   //Cos 0 to 180 deg = -sin(Angle Rad - 90deg)
                SinA = cos(AngleRad);      //Sin o to 180 deg = cos(Angle Rad - 90deg)
        }
        else {   //Angle between 180 and 360
                //Subtract 270 to shift range
                AngleDeg = AngleDeg -270.0;
                //Convert degree to radials
                AngleRad = (AngleDeg*3.141592)/180.0;

                SinA = -cos(AngleRad);      //Sin 180 to 360 deg = -cos(Angle Rad - 270deg)
                CosA = sin(AngleRad);   //Cos 180 to 360 deg = sin(Angle Rad - 270deg)
        }


        return;
}

/*--------------------------------------------------------------------
  ;[BOOGTAN2] Gets the Inverse Tangus from X/Y with the where Y can be zero or negative
  ;BoogTanX       - Input X
  ;BoogTanY       - Input Y
  ;BoogTan        - Output BOOGTAN2(X/Y)*/
void indomptableARM::GetBoogTan(float BoogTanX, float BoogTanY){


        if(BoogTanX == 0) {   // X=0 -> 0 or PI
                if(BoogTanY >= 0) {
                        BoogTan = 0.0;
                        //putchar('P');
                }else {
                        BoogTan = 3.141592;
                }
        }else {

                if(BoogTanY == 0) {   // Y=0 -> +/- Pi/2
                        if(BoogTanX > 0) {
                                BoogTan = 3.141592 / 2.0;
                        }else {
                                BoogTan = -3.141592 / 2.0;
                        }
                }else {

                        if(BoogTanY > 0) {   //BOOGTAN(X/Y)
                                BoogTan = atan((BoogTanX) / (BoogTanY));
                                //putchar('M');
                        }else  {
                                if(BoogTanX > 0) {   //BOOGTAN(X/Y) + PI   
                                        BoogTan = atan((BoogTanX) / (BoogTanY)) + 3.141592;
                                }else {              //BOOGTAN(X/Y) - PI   
                                        BoogTan = atan((BoogTanX) / (BoogTanY)) - 3.141592;
                                }
                        }
                }
        }



        return;
}

void indomptableARM::LegIK(signed int IKFeetPosX, signed int IKFeetPosY, signed int IKFeetPosZ){

        //Length between the Coxa and Feet
        IKFeetPosXZ = (int) sqrt((float)((IKFeetPosX*IKFeetPosX)+(IKFeetPosZ*IKFeetPosZ)));

        //IKSW - Length between shoulder and wrist
        IKSW = sqrt((float)(((IKFeetPosXZ-CoxaLength)*(IKFeetPosXZ-CoxaLength))+(IKFeetPosY*IKFeetPosY)));
        //IKSW2 = sqrt((float)(((IKFeetPosXZ-CoxaLength)*(IKFeetPosXZ-CoxaLength))+(IKFeetPosY*IKFeetPosY)));

        //IKA1 - Angle between SW line and the ground in rad
        GetBoogTan(IKFeetPosXZ-CoxaLength, IKFeetPosY);
        IKA1 = BoogTan;

        //IKA2 - ?
        IKA2 = acos( ((float)((FemurLength*FemurLength) - (TibiaLength*TibiaLength)) + (IKSW*IKSW)) / ((float)(2*FemurLength) * IKSW) );

        //IKFemurAngle
        IKFemurAngle = ( -(IKA1 + IKA2) * 180.0 / 3.141592 )+90;

        //IKTibiaAngle
        IKTibiaAngle = -( 90-(acos( ( (float)(FemurLength*FemurLength) + (TibiaLength*TibiaLength) - (IKSW*IKSW) ) / ((float)(2*FemurLength*TibiaLength)) )*180.0 / 3.141592) );

        //IKCoxaAngle
        GetBoogTan(IKFeetPosZ, IKFeetPosX);
        IKCoxaAngle = ((BoogTan*180.0) / 3.141592);

        //Set the Solution quality   
        if(IKSW < (float)(FemurLength+TibiaLength-30)) {
                IKSolution = TRUE;
        }
        else {
                if(IKSW < (float)(FemurLength+TibiaLength)) {
                        IKSolutionWarning = TRUE;
                }
                else {
                        IKSolutionError = TRUE;
                }
        }



        return;
}










/*
//--------------------------------------------------------------------

//  ;[SERVO DRIVER] Updates the positions of the servos    

void indomptableARM::ServoDriver(void){

        char Serout[260]={0};
        int temp = 0;

        temp = (int)( (float)(CoxaAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET0;
        sprintf(Serout, "%s #%dP%d", Serout, 0, temp);

        temp = (int)( (float)(FemurAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET4;
        sprintf(Serout, "%s #%dP%d", Serout, 4, temp);

        temp = (int)( (float)(TibiaAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET8;
        sprintf(Serout, "%s #%dP%d", Serout, 8, temp);

        temp = (int)( (float)(AnkleAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET12;
        sprintf(Serout, "%s #%dP%d", Serout, 12, temp);


        // Time and <CR>
        sprintf(Serout, "%s T%d\r", Serout, ActualGaitSpeed);

        // write to serial if connected
        if ( ser_fd_ssc )
                write(ser_fd_ssc, &Serout, sizeof(Serout));

        //printf("%s \n",Serout);
        return;
}

//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
void indomptableARM::FreeServos()
{
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
}
*/

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

