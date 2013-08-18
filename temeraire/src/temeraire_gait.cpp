#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
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

/************ DEFINES ***********/

#define False 0
#define True  1 
#define FALSE 0
#define TRUE  1

//[MIN/MAX ANGLES]
#define RRCoxa_MIN -26      //Mechanical limits of the Right Rear Leg
#define RRCoxa_MAX 74
#define RRFemur_MIN -101
#define RRFemur_MAX 95
#define RRTibia_MIN -106
#define RRTibia_MAX 77

#define RMCoxa_MIN -53      //Mechanical limits of the Right Middle Leg
#define RMCoxa_MAX 53
#define RMFemur_MIN -101
#define RMFemur_MAX 95
#define RMTibia_MIN -106
#define RMTibia_MAX 77

#define RFCoxa_MIN -58      //Mechanical limits of the Right Front Leg
#define RFCoxa_MAX 74
#define RFFemur_MIN -101
#define RFFemur_MAX 95
#define RFTibia_MIN -106
#define RFTibia_MAX 77

#define LRCoxa_MIN -74      //Mechanical limits of the Left Rear Leg
#define LRCoxa_MAX 26
#define LRFemur_MIN -95
#define LRFemur_MAX 101
#define LRTibia_MIN -77
#define LRTibia_MAX 106

#define LMCoxa_MIN -53      //Mechanical limits of the Left Middle Leg
#define LMCoxa_MAX 53
#define LMFemur_MIN -95
#define LMFemur_MAX 101
#define LMTibia_MIN -77
#define LMTibia_MAX 106

#define LFCoxa_MIN -74      //Mechanical limits of the Left Front Leg
#define LFCoxa_MAX 58
#define LFFemur_MIN -95
#define LFFemur_MAX 101
#define LFTibia_MIN -77
#define LFTibia_MAX 106

//[BODY DIMENSIONS]
#define CoxaLength 29      //Length of the Coxa [mm]
#define FemurLength 76      //Length of the Femur [mm]
//#define TibiaLength 106     //Lenght of the Tibia [mm]
#define TibiaLength 120     //NEW Lenght of the Tibia [mm]

#define FemurLength2 197      //Length of the Femur [mm]
#define TibiaLength2 158     //Lenght of the Tibia [mm] 242

#define CoxaAngle 60      //Default Coxa setup angle

#define RFOffsetX -43      //Distance X from center of the body to the Right Front coxa
#define RFOffsetZ -82      //Distance Z from center of the body to the Right Front coxa
#define RMOffsetX -63      //Distance X from center of the body to the Right Middle coxa
#define RMOffsetZ 0        //Distance Z from center of the body to the Right Middle coxa
#define RROffsetX -43      //Distance X from center of the body to the Right Rear coxa
#define RROffsetZ 82       //Distance Z from center of the body to the Right Rear coxa

#define LFOffsetX 43      //Distance X from center of the body to the Left Front coxa
#define LFOffsetZ -82     //Distance Z from center of the body to the Left Front coxa
#define LMOffsetX 63      //Distance X from center of the body to the Left Middle coxa
#define LMOffsetZ 0       //Distance Z from center of the body to the Left Middle coxa
#define LROffsetX 43      //Distance X from center of the body to the Left Rear coxa
#define LROffsetZ 82      //Distance Z from center of the body to the Left Rear coxa
//--------------------------------------------------------------------
//[REMOTE]
#define TravelDeadZone 4   //The deadzone for the analog input from the remote

#define LIMIT_HEIGHT 10





class TemeraireGait {
	public:
		TemeraireGait();
		void computeNextStep(void);

	private:
		void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
		void poseCallback(const geometry_msgs::Twist::ConstPtr& pose);
		void gaitCallback(const std_msgs::Int32::ConstPtr& gait);

		void GetSinCos (float AngleDeg);
		void GetBoogTan(float BoogTanX, float BoogTanY);

		void BodyIK(signed int PosX, signed int PosZ, signed int PosY, signed int BodyOffsetX, signed int BodyOffsetZ, signed int RotationY);
		void LegIK(signed int IKFeetPosX, signed int IKFeetPosY, signed int IKFeetPosZ);
		void LegIK2(signed long IKFeetPosX, signed long IKFeetPosY, signed long IKFeetPosZ);
		void setRotPoint(signed int z);
		void doBodyRot(void);
		void BalCalcOneLeg(double my_PosX, double my_PosZ,double my_PosY,int my_BodyOffsetX, int my_BodyOffsetZ);
		void BalanceBody(void);
		void do_IKs(void);

		void GaitSelect(void);
		void Gait(char GaitLegNr, signed int GaitPosXX, signed int GaitPosYY, signed int GaitPosZZ, signed int GaitRotYY);
		void GaitSeq(void);

		ros::NodeHandle nh;
		ros::Subscriber vel_sub;
		ros::Subscriber pose_sub;
		ros::Subscriber gait_sub;

		ros::Publisher servo_pub;
		ros::Publisher speed_pub;




		signed int ARMCoxaAngle;   
		signed int ARMFemurAngle;
		signed int ARMTibiaAngle;

		signed int X, Y, Z, Xbase, Ybase, Zbase;


		//====================================================================
		//[ANGLES]
		signed int RFCoxaAngle;   //Actual Angle of the Right Front Leg
		signed int RFFemurAngle;
		signed int RFTibiaAngle;

		signed int RMCoxaAngle;   //Actual Angle of the Right Middle Leg
		signed int RMFemurAngle;
		signed int RMTibiaAngle;

		signed int RRCoxaAngle;   //Actual Angle of the Right Rear Leg
		signed int RRFemurAngle;
		signed int RRTibiaAngle;

		signed int LFCoxaAngle;   //Actual Angle of the Left Front Leg
		signed int LFFemurAngle;
		signed int LFTibiaAngle;

		signed int LMCoxaAngle;   //Actual Angle of the Left Middle Leg
		signed int LMFemurAngle;
		signed int LMTibiaAngle;

		signed int LRCoxaAngle;   //Actual Angle of the Left Rear Leg
		signed int LRFemurAngle;
		signed int LRTibiaAngle;

		signed int horizontal_turret;
		signed int vertical_turret;
		//--------------------------------------------------------------------
		//[POSITIONS]
		signed int RFPosX;      //Actual Position of the Right Front Leg
		signed int RFPosY;
		signed int RFPosZ;

		signed int RMPosX;      //Actual Position of the Right Middle Leg
		signed int RMPosY;
		signed int RMPosZ;

		signed int RRPosX;      //Actual Position of the Right Rear Leg
		signed int RRPosY;
		signed int RRPosZ;

		signed int LFPosX;      //Actual Position of the Left Front Leg
		signed int LFPosY;
		signed int LFPosZ;

		signed int LMPosX;      //Actual Position of the Left Middle Leg
		signed int LMPosY;
		signed int LMPosZ;

		signed int LRPosX;      //Actual Position of the Left Rear Leg
		signed int LRPosY;
		signed int LRPosZ;


		//--------------------------------------------------------------------
		//[VARIABLES]
		char Index;      //Index used for freeing the servos
		char SSCDone;    //Char to check if SSC is done

		//GetSinCos
		//float AngleDeg;      //Input Angle in degrees
		float ABSAngleDeg;      //Absolute value of the Angle in Degrees
		float AngleRad;     //Angle in Radian
		float SinA;      //Output Sinus of the given Angle
		float CosA;      //Output Cosinus of the given Angle

		//GetBoogTan (Atan)
		//signed int BoogTanX;      //Input X
		//signed int BoogTanY;      //Input Y
		float BoogTan;      //Output BOOGTAN2(X/Y)

		//Body position
		signed int BodyPosX;      //Global Input for the position of the body
		signed int BodyPosY;
		signed int BodyPosZ;

		signed int BodyPosXint;
		signed int BodyPosZint; 
		signed int BodyPosYint;


		//Body Inverse Kinematics
		signed char BodyRotX;          //Global Input pitch of the body
		signed char BodyRotY;          //Global Input rotation of the body
		signed char BodyRotZ;          //Global Input roll of the body
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
		//signed int IKFeetPosX;         //Input position of the Feet X
		//signed int IKFeetPosY;         //Input position of the Feet Y
		//signed int IKFeetPosZ;         //Input Position of the Feet Z
		signed int IKFeetPosXZ;        //Length between the coxa and feet
		float IKSW;             //Length between shoulder and wrist
		float IKA1;             //Angle between SW line and the ground in rad
		float IKA2;             //?
		char IKSolution;         //Output true if the solution is possible
		char IKSolutionWarning;      //Output true if the solution is NEARLY possible
		char IKSolutionError;      //Output true if the solution is NOT possible
		signed int IKFemurAngle;       //Output Angle of Femur in degrees
		signed int IKTibiaAngle;       //Output Angle of Tibia in degrees
		signed int IKCoxaAngle;        //Output Angle of Coxa in degrees

		char ResetInitPos;
		char Mode;    //ch5 Mode switch + twostate switch H = 6 modes  
		char TestLeg;

		//--------------------------------------------------------------------
		//[GLOABAL]
		char HexOn;        //Switch to turn on Phoenix
		char TurnOff;        //Mark to turn off Phoenix
		//--------------------------------------------------------------------
		//[Balance]
		char BalanceMode;
		signed int TravelHeightY;
		signed int TotalTransX;
		signed int TotalTransZ;
		signed int TotalTransY;
		signed int TotalYBal;
		signed int TotalXBal;
		signed int TotalZBal;
		signed int TotalY;       //Total Y distance between the center of the body and the feet

		//[gait]
		char GaitType;   //Gait type
		int NomGaitSpeed;   //Nominal speed of the gait
		int ActualGaitSpeed;

		signed int LegLiftHeight;   //Current Travel height
		signed int TravelLengthX;   //Current Travel length X
		signed int TravelLengthZ;   //Current Travel length Z
		signed int TravelRotationY;   //Current Travel Rotation Y

		signed int TLDivFactor;   //Number of steps that a leg is on the floor while walking
		char NrLiftedPos;      //Number of positions that a single leg is lifted (1-3)
		char HalfLiftHeigth;      //If TRUE the outer positions of the ligted legs will be half height   

		char GaitInMotion;      //Temp to check if the gait is in motion
		char StepsInGait;   //Number of steps in gait
		char LastLeg;      //TRUE when the current leg is the last leg of the sequence
		char GaitStep;   //Actual Gait step

		signed int RFGaitLegNr;   //Init position of the leg
		signed int RMGaitLegNr;   //Init position of the leg
		signed int RRGaitLegNr;   //Init position of the leg
		signed int LFGaitLegNr;   //Init position of the leg
		signed int LMGaitLegNr;   //Init position of the leg
		signed int LRGaitLegNr;   //Init position of the leg

		//char GaitLegNr;   //Input Number of the leg
		signed int TravelMulti;   //Multiplier for the length of the step

		signed int RFGaitPosX;   //Relative position corresponding to the Gait
		signed int RFGaitPosY;
		signed int RFGaitPosZ;
		signed int RFGaitRotY;   //Relative rotation corresponding to the Gait

		signed int RMGaitPosX;
		signed int RMGaitPosY;
		signed int RMGaitPosZ;
		signed int RMGaitRotY;

		signed int RRGaitPosX;
		signed int RRGaitPosY;
		signed int RRGaitPosZ;
		signed int RRGaitRotY;

		signed int LFGaitPosX;
		signed int LFGaitPosY;
		signed int LFGaitPosZ;
		signed int LFGaitRotY;

		signed int LMGaitPosX;
		signed int LMGaitPosY;
		signed int LMGaitPosZ;
		signed int LMGaitRotY;

		signed int LRGaitPosX;
		signed int LRGaitPosY;
		signed int LRGaitPosZ;
		signed int LRGaitRotY;

		signed int GaitPosX;   //In-/Output Pos X of feet
		signed int GaitPosY;   //In-/Output Pos Y of feet
		signed int GaitPosZ;   //In-/Output Pos Z of feet
		signed int GaitRotY;   //In-/Output Rotation Y of feet

		char starting;
		char sleeping;
		char display1, display2, display3, display4;
		int entier;
		signed int headAngle;
		signed int down_leg_step;

		signed int RotPoint; 
		char mylegnumber;
		int leg_sensor_ON;

};

TemeraireGait::TemeraireGait()
{
	vel_sub = nh.subscribe<geometry_msgs::Twist>("/TEMERAIRE/cmd_vel", 5, &TemeraireGait::velCallback, this);
	pose_sub = nh.subscribe<geometry_msgs::Twist>("/TEMERAIRE/body_pose", 5, &TemeraireGait::poseCallback, this);
	gait_sub = nh.subscribe<std_msgs::Int32>("/TEMERAIRE/gait", 5, &TemeraireGait::gaitCallback, this);

	servo_pub = nh.advertise < temeraire::SSC32_Servo > ("/TEMERAIRE/ssc32_comm", 3);
	speed_pub = nh.advertise < std_msgs::Int32 > ("/TEMERAIRE/ssc32_speed", 3);






	ARMCoxaAngle = 0;   
	ARMFemurAngle = 0;
	ARMTibiaAngle = 0;


	//====================================================================
	//[ANGLES]
	RFCoxaAngle = 0;   //Actual Angle of the Right Front Leg
	RFFemurAngle = 0;
	RFTibiaAngle = 0;

	RMCoxaAngle = 0;   //Actual Angle of the Right Middle Leg
	RMFemurAngle = 0;
	RMTibiaAngle = 0;

	RRCoxaAngle = 0;   //Actual Angle of the Right Rear Leg
	RRFemurAngle = 0;
	RRTibiaAngle = 0;

	LFCoxaAngle = 0;   //Actual Angle of the Left Front Leg
	LFFemurAngle = 0;
	LFTibiaAngle = 0;

	LMCoxaAngle = 0;   //Actual Angle of the Left Middle Leg
	LMFemurAngle = 0;
	LMTibiaAngle = 0;

	LRCoxaAngle = 0;   //Actual Angle of the Left Rear Leg
	LRFemurAngle = 0;
	LRTibiaAngle = 0;

	//--------------------------------------------------------------------
	//[POSITIONS]
	RFPosX = 0;      //Actual Position of the Right Front Leg
	RFPosY = 0;
	RFPosZ = 0;

	RMPosX = 0;      //Actual Position of the Right Middle Leg
	RMPosY = 0;
	RMPosZ = 0;

	RRPosX = 0;      //Actual Position of the Right Rear Leg
	RRPosY = 0;
	RRPosZ = 0;

	LFPosX = 0;      //Actual Position of the Left Front Leg
	LFPosY = 0;
	LFPosZ = 0;

	LMPosX = 0;      //Actual Position of the Left Middle Leg
	LMPosY = 0;
	LMPosZ = 0;

	LRPosX = 0;      //Actual Position of the Left Rear Leg
	LRPosY = 0;
	LRPosZ = 0;

	//Body position
	BodyPosX = 0;      //Global Input for the position of the body
	BodyPosY = 0;
	BodyPosZ = 0;

	BodyPosXint = 0;
	BodyPosZint = 0; 
	BodyPosYint = 0;


	//Body Inverse Kinematics
	BodyRotX = 0;          //Global Input pitch of the body
	BodyRotY = 0;          //Global Input rotation of the body
	BodyRotZ = 0;          //Global Input roll of the body

	TravelHeightY = 0;
	TotalTransX = 0;
	TotalTransZ = 0;
	TotalTransY = 0;
	TotalYBal = 0;
	TotalXBal = 0;
	TotalZBal = 0;
	TotalY = 0;       //Total Y distance between the center of the body and the feet

	RFGaitLegNr = 0;   //Init position of the leg
	RMGaitLegNr = 0;   //Init position of the leg
	RRGaitLegNr = 0;   //Init position of the leg
	LFGaitLegNr = 0;   //Init position of the leg
	LMGaitLegNr = 0;   //Init position of the leg
	LRGaitLegNr = 0;   //Init position of the leg

	//char GaitLegNr;   //Input Number of the leg
	TravelMulti;   //Multiplier for the length of the step

	RFGaitPosX = 0;   //Relative position corresponding to the Gait
	RFGaitPosY = 0;
	RFGaitPosZ = 0;
	RFGaitRotY = 0;   //Relative rotation corresponding to the Gait

	RMGaitPosX = 0;
	RMGaitPosY = 0;
	RMGaitPosZ = 0;
	RMGaitRotY = 0;

	RRGaitPosX = 0;
	RRGaitPosY = 0;
	RRGaitPosZ = 0;
	RRGaitRotY = 0;

	LFGaitPosX = 0;
	LFGaitPosY = 0;
	LFGaitPosZ = 0;
	LFGaitRotY = 0;

	LMGaitPosX = 0;
	LMGaitPosY = 0;
	LMGaitPosZ = 0;
	LMGaitRotY = 0;

	LRGaitPosX = 0;
	LRGaitPosY = 0;
	LRGaitPosZ = 0;
	LRGaitRotY = 0;

	GaitPosX = 0;   //In-/Output Pos X of feet
	GaitPosY = 0;   //In-/Output Pos Y of feet
	GaitPosZ = 0;   //In-/Output Pos Z of feet
	GaitRotY = 0;   //In-/Output Rotation Y of feet

	starting =0;
	sleeping =0;
	down_leg_step = 2;

	RotPoint = 0;
	mylegnumber = 0;
	leg_sensor_ON = 0;



        //Gait
        GaitType = 0;
        BalanceMode = 0;
        LegLiftHeight = 50;
        GaitStep = 1;
        Mode = 1;

        //What leg is active variable 1-6

        //Whatleg = 0

        //This resets the Init positions of each leg when they are modified with two leg mode
        ResetInitPos = False;

        //GaitSelect();




        BodyPosY = 0;
        TravelLengthX = 0;
        TravelLengthZ = 0;
        TravelRotationY = 0;

                TravelLengthX = 0;   //Current Travel length X
                TravelLengthZ = 0;   //Current Travel length Z
                TravelRotationY = 0;   //Current Travel Rotation Y


		/* VRAIE POSITION */ 

		RFPosX = 60;      //Start positions of the Right Front leg

		RFPosY = 25;

		RFPosZ = -71;



		RMPosX = 90;   //Start positions of the Right Middle leg

		RMPosY = 25;

		RMPosZ = -10;	



		RRPosX = 73;    //Start positions of the Right Rear leg

		RRPosY = 25;

		RRPosZ = 51;



		LFPosX = 60;      //Start positions of the Left Front leg

		LFPosY = 25;

		LFPosZ = -71;



		LMPosX = 90;   //Start positions of the Left Middle leg

		LMPosY = 25;

		LMPosZ = -10;



		LRPosX = 73;      //Start positions of the Left Rear leg

		LRPosY = 25;

		LRPosZ = 51;







        LRGaitPosY = 0;
        RFGaitPosY = 0;
        LMGaitPosY = 0;
        RRGaitPosY = 0;
        LFGaitPosY = 0;
        RMGaitPosY = 0;



		// RotPoint = 82; //(arriere du robot)
		RotPoint = 0; // (center du robot)

		//Body Positions
		BodyPosX = 0;
		BodyPosY = 0;
		BodyPosZ = 0;
		BodyPosXint = 0;
		BodyPosZint = 0;
		BodyPosYint = 0;

		//Body Rotations
		BodyRotX = 0;
		BodyRotY = 0;
		BodyRotZ = 0;



		// 178 / 275

		X = 0;
		Y = 0;
		Z = 0;


		Xbase = 200;
		Ybase = 108;
		Zbase = -80;

		BodyPosYint = 65;
		NomGaitSpeed = 500;





	GaitType = 1;
	TravelLengthZ = 0;


	GaitSelect();




}


/*****************************************/
/******** UTILITY FUNCTIONS ! ************/
/*****************************************/


/**--------------------------------------------------------------------
  [GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
  AngleDeg    - Input Angle in degrees
  SinA        - Output Sinus of AngleDeg
  CosA        - Output Cosinus of AngleDeg*/
void TemeraireGait::GetSinCos (float AngleDeg) {

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
void TemeraireGait::GetBoogTan(float BoogTanX, float BoogTanY){


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




/*****************************************/
/********* IK FUNCTIONS ! ****************/
/*****************************************/

/**--------------------------------------------------------------------
  ;[BODY INVERSE KINEMATICS]
  ;BodyRotX         - Global Input pitch of the body
  ;BodyRotY         - Global Input rotation of the body
  ;BodyRotZ         - Global Input roll of the body
  ;RotationY         - Input Rotation for the gait
  ;PosX            - Input position of the feet X
  ;PosZ            - Input position of the feet Z
  ;BodyOffsetX      - Input Offset betweeen the body and Coxa X
  ;BodyOffsetZ      - Input Offset betweeen the body and Coxa Z
  ;SinB                - Sin buffer for BodyRotX
  ;CosB              - Cos buffer for BodyRotX
  ;SinG                - Sin buffer for BodyRotZ
  ;CosG              - Cos buffer for BodyRotZ
  ;BodyIKPosX         - Output Position X of feet with Rotation
  ;BodyIKPosY         - Output Position Y of feet with Rotation
  ;BodyIKPosZ         - Output Position Z of feet with Rotation */
void TemeraireGait::BodyIK(signed int PosX, signed int PosZ, signed int PosY, signed int BodyOffsetX, signed int BodyOffsetZ, signed int RotationY) {

	//Calculating totals from center of the body to the feet
	TotalZ = BodyOffsetZ+PosZ;
	TotalX = BodyOffsetX+PosX;
	//PosY are equal to a "TotalY"


	//Successive global rotation matrix:
	//Math shorts for rotation: Alfa (A) = Xrotate, Beta (B) = Zrotate, Gamma (G) = Yrotate
	//Sinus Alfa = sinA, cosinus Alfa = cosA. and so on...

	//First calculate sinus and cosinus for each rotation:
	GetSinCos((float)(BodyRotX+TotalXBal));
	SinG = SinA;
	CosG = CosA;
	GetSinCos((float)(BodyRotZ+TotalZBal));
	SinB = SinA;
	CosB = CosA;
	GetSinCos((float)(BodyRotY+RotationY+TotalYBal));

	//Calcualtion of rotation matrix:
	BodyIKPosX = TotalX- (signed int)((float)(TotalX)*CosA*CosB - (float)(TotalZ)*CosB*SinA + (float)(PosY)*SinB);
	BodyIKPosZ = TotalZ- (signed int)((float)(TotalX)*CosG*SinA + (float)(TotalX)*CosA*SinB*SinG +(float)(TotalZ)*CosA*CosG-(float)(TotalZ)*SinA*SinB*SinG-(float)(PosY)*CosB*SinG);
	BodyIKPosY = PosY -  (signed int)((float)(TotalX)*SinA*SinG - (float)(TotalX)*CosA*CosG*SinB + (float)(TotalZ)*CosA*SinG + (float)(TotalZ)*CosG*SinA*SinB + (float)(PosY)*CosB*CosG);



	return;
}

/*--------------------------------------------------------------------
  ;[LEG INVERSE KINEMATICS] Calculates the angles of the tibia and femur for the given position of the feet
  ;IKFeetPosX         - Input position of the Feet X
  ;IKFeetPosY         - Input position of the Feet Y
  ;IKFeetPosZ         - Input Position of the Feet Z
  ;IKSolution         - Output true IF the solution is possible
  ;IKSolutionWarning    - Output true IF the solution is NEARLY possible
  ;IKSolutionError   - Output true IF the solution is NOT possible
  ;IKFemurAngle      - Output Angle of Femur in degrees
  ;IKTibiaAngle      - Output Angle of Tibia in degrees
  ;IKCoxaAngle      - Output Angle of Coxa in degrees*/
void TemeraireGait::LegIK(signed int IKFeetPosX, signed int IKFeetPosY, signed int IKFeetPosZ){


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

/*--------------------------------------------------------------------
  ;[LEG INVERSE KINEMATICS] Calculates the angles of the tibia and femur for the given position of the feet
  ;IKFeetPosX         - Input position of the Feet X
  ;IKFeetPosY         - Input position of the Feet Y
  ;IKFeetPosZ         - Input Position of the Feet Z
  ;IKSolution         - Output true IF the solution is possible
  ;IKSolutionWarning    - Output true IF the solution is NEARLY possible
  ;IKSolutionError   - Output true IF the solution is NOT possible
  ;IKFemurAngle      - Output Angle of Femur in degrees
  ;IKTibiaAngle      - Output Angle of Tibia in degrees
  ;IKCoxaAngle      - Output Angle of Coxa in degrees*/
void TemeraireGait::LegIK2(signed long IKFeetPosX, signed long IKFeetPosY, signed long IKFeetPosZ){


	//Length between the Coxa and Feet
	IKFeetPosXZ = (int) sqrt(((IKFeetPosX*IKFeetPosX)+(IKFeetPosZ*IKFeetPosZ)));

	//IKSW - Length between shoulder and wrist
	IKSW = sqrt((((long)((long)IKFeetPosXZ-CoxaLength)*(long)((long)IKFeetPosXZ-CoxaLength))+(IKFeetPosY*IKFeetPosY)));

	//IKA1 - Angle between SW line and the ground in rad
	GetBoogTan(IKFeetPosXZ-CoxaLength, IKFeetPosY);
	IKA1 = BoogTan;

	//IKA2 - ?
	IKA2 = acos( ((float)(((long)FemurLength2*(long)FemurLength2) - ((long)TibiaLength2*(long)TibiaLength2)) + (IKSW*IKSW)) / ((float)(2*FemurLength2) * IKSW) );

	//IKFemurAngle
	IKFemurAngle = ( -(IKA1 + IKA2) * 180.0 / 3.141592 )+90;

	//IKTibiaAngle
	IKTibiaAngle = -( 90-(acos( ( (float)(FemurLength2*FemurLength2) + (TibiaLength2*TibiaLength2) - (IKSW*IKSW) ) / ((float)(2*FemurLength2*TibiaLength2)) )*180.0 / 3.141592) );

	//IKCoxaAngle
	GetBoogTan(IKFeetPosZ, IKFeetPosX);
	IKCoxaAngle = ((BoogTan*180.0) / 3.141592);

	//Set the Solution quality   
	if(IKSW < (float)(FemurLength2+TibiaLength2-30)) {
		IKSolution = TRUE;
	}
	else {
		if(IKSW < (float)(FemurLength2+TibiaLength2)) {
			IKSolutionWarning = TRUE;
		}
		else {
			IKSolutionError = TRUE;
		}
	}


	return;
}

void TemeraireGait::setRotPoint(signed int z) {

	RotPoint = z;

	return;
}

void TemeraireGait::doBodyRot(void) {
	// Degres : BodyRot
	signed int PosZ1;
	// Rotation suivant Y
	GetSinCos( (float)BodyRotY );
	// BodyPosZ = cos( BodyRotY ) * ( (RotPoint / cos( BodyRotY )) - RotPoint);
	PosZ1 = (signed int)( ((float)RotPoint) * ( 1.0 - CosA ));
	BodyPosX = BodyPosXint + (signed int)( ((float)RotPoint) *  SinA);

	// Rotation suivant X
	GetSinCos( (float)BodyRotX );
	BodyPosZ = BodyPosZint + PosZ1 + (signed int)( ((float)RotPoint) * ( 1.0 - CosA ));
	BodyPosY = BodyPosYint - (signed int)( ((float)RotPoint) *  SinA);

	return;
}

//;--------------------------------------------------------------------
//;[BalCalcOneLeg]
void TemeraireGait::BalCalcOneLeg(double my_PosX, double my_PosZ,double my_PosY,int my_BodyOffsetX, int my_BodyOffsetZ) {
	//;Calculating totals from center of the body to the feet
	TotalZ = my_BodyOffsetZ+my_PosZ;
	TotalX = my_BodyOffsetX+my_PosX;
	TotalY =  100 + my_PosY; //' using the value 150 to lower the centerpoint of rotation 'BodyPosY +
	TotalTransY = TotalTransY + my_PosY;
	TotalTransZ = TotalTransZ + TotalZ;
	TotalTransX = TotalTransX + TotalX;
	GetBoogTan( TotalX, TotalZ);
	TotalYBal = TotalYBal + (int)((BoogTan*180.0) / 3.141592);
	GetBoogTan (TotalX, TotalY);
	TotalZBal = TotalZBal + (int)((BoogTan*180.0) / 3.141592);
	GetBoogTan (TotalZ, TotalY);
	TotalXBal = TotalXBal + (int)((BoogTan*180.0) / 3.141592);
	return;
}

//;--------------------------------------------------------------------
//;[BalanceBody]
void TemeraireGait::BalanceBody(void) {
	TotalTransZ = TotalTransZ/6;
	TotalTransX = TotalTransX/6;
	TotalTransY = TotalTransY/6;
	if( TotalYBal < -180 )  //'Tangens fix caused by +/- 180 deg
		TotalYBal = TotalYBal + 360;
	if( TotalZBal < -180 )  //'Tangens fix caused by +/- 180 deg
		TotalZBal = TotalZBal + 360;
	if( TotalXBal < -180 )  //'Tangens fix caused by +/- 180 deg
		TotalXBal = TotalXBal + 360;

	//;Balance rotation
	TotalYBal = TotalYBal/6;
	TotalXBal = TotalXBal/6;
	TotalZBal = -TotalZBal/6;

	//;Balance translation
	LFGaitPosZ = LFGaitPosZ - TotalTransZ;
	LMGaitPosZ = LMGaitPosZ - TotalTransZ;
	LRGaitPosZ = LRGaitPosZ - TotalTransZ;
	RFGaitPosZ = RFGaitPosZ - TotalTransZ;
	RMGaitPosZ = RMGaitPosZ - TotalTransZ;
	RRGaitPosZ = RRGaitPosZ - TotalTransZ;

	LFGaitPosX = LFGaitPosX - TotalTransX;
	LMGaitPosX = LMGaitPosX - TotalTransX;
	LRGaitPosX = LRGaitPosX - TotalTransX;
	RFGaitPosX = RFGaitPosX - TotalTransX;
	RMGaitPosX = RMGaitPosX - TotalTransX;
	RRGaitPosX = RRGaitPosX - TotalTransX ;

	LFGaitPosY = LFGaitPosY - TotalTransY;
	LMGaitPosY = LMGaitPosY - TotalTransY;
	LRGaitPosY = LRGaitPosY - TotalTransY;
	RFGaitPosY = RFGaitPosY - TotalTransY;
	RMGaitPosY = RMGaitPosY - TotalTransY;
	RRGaitPosY = RRGaitPosY - TotalTransY;
	return;
}

void TemeraireGait::do_IKs(void) {

	//Right Front leg
	BodyIK(-RFPosX+BodyPosX+RFGaitPosX, RFPosZ+BodyPosZ+RFGaitPosZ,RFPosY+BodyPosY+RFGaitPosY, (signed int)RFOffsetX, (signed int)RFOffsetZ, (signed int)RFGaitRotY);
	LegIK(RFPosX-BodyPosX+BodyIKPosX-RFGaitPosX, RFPosY+BodyPosY-BodyIKPosY+RFGaitPosY, RFPosZ+BodyPosZ-BodyIKPosZ+RFGaitPosZ);
	RFCoxaAngle  = IKCoxaAngle + CoxaAngle; //Angle for the basic setup for the front leg   
	RFFemurAngle = IKFemurAngle;
	RFTibiaAngle = IKTibiaAngle;

	//Right Middle leg
	BodyIK(-RMPosX+BodyPosX+RMGaitPosX, RMPosZ+BodyPosZ+RMGaitPosZ,RMPosY+BodyPosY+RMGaitPosY, (signed int)RMOffsetX, (signed int)RMOffsetZ, (signed int)RMGaitRotY);
	LegIK(RMPosX-BodyPosX+BodyIKPosX-RMGaitPosX, RMPosY+BodyPosY-BodyIKPosY+RMGaitPosY, RMPosZ+BodyPosZ-BodyIKPosZ+RMGaitPosZ);
	RMCoxaAngle  = IKCoxaAngle;
	RMFemurAngle = IKFemurAngle;
	RMTibiaAngle = IKTibiaAngle;

	//Right Rear leg
	BodyIK(-RRPosX+BodyPosX+RRGaitPosX, RRPosZ+BodyPosZ+RRGaitPosZ,RRPosY+BodyPosY+RRGaitPosY, (signed int)RROffsetX, (signed int)RROffsetZ, (signed int)RRGaitRotY);
	LegIK(RRPosX-BodyPosX+BodyIKPosX-RRGaitPosX, RRPosY+BodyPosY-BodyIKPosY+RRGaitPosY, RRPosZ+BodyPosZ-BodyIKPosZ+RRGaitPosZ);
	RRCoxaAngle  = IKCoxaAngle - CoxaAngle; //Angle for the basic setup for the front leg   
	RRFemurAngle = IKFemurAngle;
	RRTibiaAngle = IKTibiaAngle;

	//Left Front leg
	BodyIK(LFPosX-BodyPosX+LFGaitPosX, LFPosZ+BodyPosZ+LFGaitPosZ,LFPosY+BodyPosY+LFGaitPosY, (signed int)LFOffsetX, (signed int)LFOffsetZ, (signed int)LFGaitRotY);
	LegIK(LFPosX+BodyPosX-BodyIKPosX+LFGaitPosX, LFPosY+BodyPosY-BodyIKPosY+LFGaitPosY, LFPosZ+BodyPosZ-BodyIKPosZ+LFGaitPosZ);
	LFCoxaAngle  = IKCoxaAngle + CoxaAngle; //Angle for the basic setup for the front leg   
	LFFemurAngle = IKFemurAngle;
	LFTibiaAngle = IKTibiaAngle;

	//Left Middle leg
	BodyIK(LMPosX-BodyPosX+LMGaitPosX, LMPosZ+BodyPosZ+LMGaitPosZ,LMPosY+BodyPosY+LMGaitPosY, (signed int)LMOffsetX, (signed int)LMOffsetZ, (signed int)LMGaitRotY);
	LegIK(LMPosX+BodyPosX-BodyIKPosX+LMGaitPosX, LMPosY+BodyPosY-BodyIKPosY+LMGaitPosY, LMPosZ+BodyPosZ-BodyIKPosZ+LMGaitPosZ);
	LMCoxaAngle  = IKCoxaAngle;
	LMFemurAngle = IKFemurAngle;
	LMTibiaAngle = IKTibiaAngle;

	//Left Rear leg
	BodyIK(LRPosX-BodyPosX+LRGaitPosX, LRPosZ+BodyPosZ+LRGaitPosZ,LRPosY+BodyPosY+LRGaitPosY, (signed int)LROffsetX, (signed int)LROffsetZ, (signed int)LRGaitRotY);
	LegIK(LRPosX+BodyPosX-BodyIKPosX+LRGaitPosX, LRPosY+BodyPosY-BodyIKPosY+LRGaitPosY, LRPosZ+BodyPosZ-BodyIKPosZ+LRGaitPosZ);
	LRCoxaAngle  = IKCoxaAngle - CoxaAngle; //Angle for the basic setup for the front leg   
	LRFemurAngle = IKFemurAngle;
	LRTibiaAngle = IKTibiaAngle;
}


/*****************************/
/****** GAIT FUNCTIONS *******/
/*****************************/

/*--------------------------------------------------------------------
  [GAIT Select] */
void TemeraireGait::GaitSelect(void) {
	//Gait selector
	if (GaitType == 0) { //Ripple Gait 6 steps
		LRGaitLegNr = 1;
		RFGaitLegNr = 2;   
		LMGaitLegNr = 3;    
		RRGaitLegNr = 4;    
		LFGaitLegNr = 5;    
		RMGaitLegNr = 6;

		NrLiftedPos = 1;
		HalfLiftHeigth = FALSE;    
		TLDivFactor = 4;    
		StepsInGait = 6;
		NomGaitSpeed = 250;
	} 

	if (GaitType == 1) { //Ripple Gait 12 steps
		LRGaitLegNr = 1;
		RFGaitLegNr = 3;
		LMGaitLegNr = 5;
		RRGaitLegNr = 7;
		LFGaitLegNr = 9;
		RMGaitLegNr = 11;

		NrLiftedPos = 3;
		HalfLiftHeigth = TRUE;
		TLDivFactor = 8;    
		StepsInGait = 12;   
		NomGaitSpeed = 250;
	}

	if (GaitType == 2) { //Quadripple 9 steps
		LRGaitLegNr = 1;   
		RFGaitLegNr = 2;
		LMGaitLegNr = 4;    
		RRGaitLegNr = 5;
		LFGaitLegNr = 7;
		RMGaitLegNr = 8;

		NrLiftedPos = 2;
		HalfLiftHeigth = FALSE;   
		TLDivFactor = 6;    
		StepsInGait = 9;      
		NomGaitSpeed = 250;
	}   

	if (GaitType == 3) { //Tripod 4 steps
		LRGaitLegNr = 3;   
		RFGaitLegNr = 1;
		LMGaitLegNr = 1;
		RRGaitLegNr = 1;
		LFGaitLegNr = 3;
		RMGaitLegNr = 3;

		NrLiftedPos = 1;
		HalfLiftHeigth = FALSE;   
		TLDivFactor = 2;    
		StepsInGait = 4;      
		NomGaitSpeed = 250;
	}

	if (GaitType == 4) { //Tripod 6 steps
		LRGaitLegNr = 4;   
		RFGaitLegNr = 1;
		LMGaitLegNr = 1;
		RRGaitLegNr = 1;
		LFGaitLegNr = 4;
		RMGaitLegNr = 4;

		NrLiftedPos = 2;
		HalfLiftHeigth = FALSE;   
		TLDivFactor = 4;    
		StepsInGait = 6;      
		NomGaitSpeed = 250;
	}

	if (GaitType == 5) { //Tripod 8 steps
		LRGaitLegNr = 5;
		RFGaitLegNr = 1;
		LMGaitLegNr = 1;
		RRGaitLegNr = 1;
		LFGaitLegNr = 5;
		RMGaitLegNr = 5;

		NrLiftedPos = 3;
		HalfLiftHeigth = TRUE;   
		TLDivFactor = 4;    
		StepsInGait = 8;      
		NomGaitSpeed = 250;
	}

	if (GaitType == 6) { //Wave 12 steps
		LRGaitLegNr = 7;
		RFGaitLegNr = 1;
		LMGaitLegNr = 9;
		RRGaitLegNr = 5;
		LFGaitLegNr = 11;
		RMGaitLegNr = 3;

		NrLiftedPos = 1;
		HalfLiftHeigth = FALSE;   
		TLDivFactor = 10;    
		StepsInGait = 12;      
		NomGaitSpeed = 250;
	}   

	if(GaitType == 7) { //Wave 18 steps
		LRGaitLegNr = 10; 
		RFGaitLegNr = 1;
		LMGaitLegNr = 13;
		RRGaitLegNr = 7;
		LFGaitLegNr = 16;
		RMGaitLegNr = 4;

		NrLiftedPos = 2;
		HalfLiftHeigth = FALSE;   
		TLDivFactor = 16;    
		StepsInGait = 18;      
		NomGaitSpeed = 250;
	}

	if(GaitType == 8) { // TEST
		LRGaitLegNr = 3;
		RFGaitLegNr = 1;
		LMGaitLegNr = 11;
		RRGaitLegNr = 9;
		LFGaitLegNr = 7;
		RMGaitLegNr = 5;

		NrLiftedPos = 1;
		HalfLiftHeigth = FALSE;
		TLDivFactor = 10;
		StepsInGait = 12;
		NomGaitSpeed = 250;
	}


	if(GaitType == 9) { // 4 legs
		LRGaitLegNr = 1; 
		RRGaitLegNr = 13;
		LFGaitLegNr = 9;
		RFGaitLegNr = 21;

		RMGaitLegNr = 200;
		LMGaitLegNr = 200;

		NrLiftedPos = 3;
		HalfLiftHeigth = FALSE;   
		TLDivFactor = 20;    
		StepsInGait = 24;      
		NomGaitSpeed = 150;
	}

	if(GaitType == 10) { //Wave 18 steps
		LRGaitLegNr = 1; 
		RFGaitLegNr = 2; 
		LMGaitLegNr = 8; 
		RRGaitLegNr = 15; 
		LFGaitLegNr = 12; 
		RMGaitLegNr = 18; 

		NrLiftedPos = 2;
		HalfLiftHeigth = FALSE;   
		TLDivFactor = 18;    
		StepsInGait = 20;      
		NomGaitSpeed = 400;
	}

	if(GaitType == 11) { //Wave 14 steps
		LRGaitLegNr = 5; 
		RFGaitLegNr = 2;
		RRGaitLegNr = 12; 
		LFGaitLegNr = 9; 

		RMGaitLegNr = 16;
		LMGaitLegNr = 20;

		NrLiftedPos = 2;
		HalfLiftHeigth = FALSE;   
		TLDivFactor = 12;    
		StepsInGait = 14;      
		NomGaitSpeed = 400;
	}

	ActualGaitSpeed = NomGaitSpeed;

	return;
}

/**--------------------------------------------------------------------
 * PEUT ETRE UTILISER CABS AU LIEU DE ABS
 [GAIT]*/
void TemeraireGait::Gait(char GaitLegNr, signed int GaitPosXX, signed int GaitPosYY, signed int GaitPosZZ, signed int GaitRotYY) {

	if(leg_sensor_ON == 1) {
	}
	else {

		if (Mode == 3) {
			if (TestLeg == GaitLegNr) {
				GaitPosX = TravelLengthX;     
				GaitPosY = -TravelHeightY;
				GaitPosZ = TravelLengthZ;
				GaitRotY = 0;
			}
		}
		else {
			//Check IF the Gait is in motion
			if( (abs(TravelLengthX)>TravelDeadZone) || (abs(TravelLengthZ)>TravelDeadZone) || (abs(TravelRotationY)>TravelDeadZone) ) {
				//putchar('J');
				GaitInMotion = 1;
			}
			else {
				GaitInMotion = 0;
			}

			//Leg middle up position
			//Gait in motion                                            Gait NOT in motion, return to home position
			if ((GaitInMotion && (NrLiftedPos==1 || NrLiftedPos==3) && GaitStep==GaitLegNr) || (GaitInMotion==FALSE && GaitStep==GaitLegNr && ((abs(GaitPosXX)>2) || (abs(GaitPosZZ)>2) || (abs(GaitRotYY)>2)))) {   //Up
				GaitPosX = 0;
				GaitPosY = -LegLiftHeight;
				GaitPosZ = 0;
				GaitRotY = 0;

			}
			else {

				//Optional Half heigth Rear
				if (((NrLiftedPos==2 && GaitStep==GaitLegNr) || (NrLiftedPos==3 && (GaitStep==(GaitLegNr-1) || GaitStep==GaitLegNr+(StepsInGait-1)))) && GaitInMotion) {
					GaitPosX = -TravelLengthX/2;
					GaitPosY = -LegLiftHeight/((signed int)HalfLiftHeigth+1);
					GaitPosZ = -TravelLengthZ/2;
					GaitRotY = -TravelRotationY/2;

				}
				else {

					//Optional half heigth front
					if ((NrLiftedPos>=2) && (GaitStep==GaitLegNr+1 || GaitStep==GaitLegNr-(StepsInGait-1)) && GaitInMotion) {
						GaitPosX = TravelLengthX/2;
						GaitPosY = -LegLiftHeight/((signed int)HalfLiftHeigth+1);
						GaitPosZ = TravelLengthZ/2;
						GaitRotY = TravelRotationY/2;

					}
					else {     

						//Leg front down position
						if ((GaitStep==GaitLegNr+NrLiftedPos || GaitStep==GaitLegNr-(StepsInGait-NrLiftedPos)) && GaitPosYY<0) {

							GaitPosX = TravelLengthX/2;
							GaitPosY = 0;
							GaitPosZ = TravelLengthZ/2;
							GaitRotY = TravelRotationY/2;
							//printf("Leg go down \n");
						}
						//Move body forward     
						else {

							GaitPosX = GaitPosXX - (TravelLengthX/TLDivFactor);     
							GaitPosY = 0;
							GaitPosZ = GaitPosZZ - (TravelLengthZ/TLDivFactor);
							GaitRotY = GaitRotYY - (TravelRotationY/TLDivFactor);
							//printf("Move body \n");
						}
					}
				}

			}
		}
		/*
		   printf("leg_on_floor = %d \n",leg_on_floor);
		   printf("GaitStep = %d \n",GaitStep); 
		 */
		//Advance to the next step 
		if (LastLeg) {   //The last leg in this step
			GaitStep = GaitStep+1;
			if (GaitStep>StepsInGait) {
				GaitStep = 1;
			}
			//printf(" \n");
		}

		return;

	}
}


/**--------------------------------------------------------------------
  [GAIT Sequence]*/
void TemeraireGait::GaitSeq(void) {
	//Calculate Gait sequence

	LastLeg = FALSE;
	Gait(LRGaitLegNr, LRGaitPosX, LRGaitPosY, LRGaitPosZ, LRGaitRotY);

	LRGaitPosX = GaitPosX;
	LRGaitPosY = GaitPosY;// + LRGaitPosY_adapt;
	LRGaitPosZ = GaitPosZ;
	LRGaitRotY = GaitRotY;   

	Gait(RFGaitLegNr, RFGaitPosX, RFGaitPosY, RFGaitPosZ, RFGaitRotY);
	RFGaitPosX = GaitPosX;
	RFGaitPosY = GaitPosY;// + RFGaitPosY_adapt;
	RFGaitPosZ = GaitPosZ;
	RFGaitRotY = GaitRotY;

	Gait(LMGaitLegNr, LMGaitPosX, LMGaitPosY, LMGaitPosZ, LMGaitRotY);
	LMGaitPosX = GaitPosX;
	LMGaitPosY = GaitPosY;// + LMGaitPosY_adapt;
	LMGaitPosZ = GaitPosZ;
	LMGaitRotY = GaitRotY;   

	Gait(RRGaitLegNr, RRGaitPosX, RRGaitPosY, RRGaitPosZ, RRGaitRotY);
	RRGaitPosX = GaitPosX;
	RRGaitPosY = GaitPosY;// + RRGaitPosY_adapt;
	RRGaitPosZ = GaitPosZ;
	RRGaitRotY = GaitRotY;   

	Gait(LFGaitLegNr, LFGaitPosX, LFGaitPosY, LFGaitPosZ, LFGaitRotY);
	LFGaitPosX = GaitPosX;
	LFGaitPosY = GaitPosY;// + LFGaitPosY_adapt;
	LFGaitPosZ = GaitPosZ;
	LFGaitRotY = GaitRotY;   
	//printf("LFGaitPosY = %d \n",LFGaitPosY);  

	LastLeg = TRUE;
	Gait(RMGaitLegNr, RMGaitPosX, RMGaitPosY, RMGaitPosZ, RMGaitRotY);
	RMGaitPosX = GaitPosX;
	RMGaitPosY = GaitPosY;// + RMGaitPosY_adapt;
	RMGaitPosZ = GaitPosZ;
	RMGaitRotY = GaitRotY;

	//printf("leg_on_floor = %d \n",leg_on_floor);
	//printf("GaitStep = %d \n",GaitStep);   
	return;
}








void TemeraireGait::velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
        TravelLengthX = vel->linear.y * 200;
        TravelLengthZ = vel->linear.x * 200;
        TravelRotationY = vel->angular.z * 100;
}

void TemeraireGait::poseCallback(const geometry_msgs::Twist::ConstPtr& pose)
{
                //Body Positions
                BodyPosXint = pose->linear.y * 150;
                BodyPosZint = pose->linear.x * 150;
                BodyPosYint = 80 + pose->linear.z * 150;

                //Body Rotations
                BodyRotX = pose->angular.y * 90;
                BodyRotY = pose->angular.z * 90;
                BodyRotZ = pose->angular.x * 90;
}


void TemeraireGait::gaitCallback(const std_msgs::Int32::ConstPtr& gait)
{
	GaitType = gait->data;
	GaitSelect();
}

void TemeraireGait::computeNextStep(void)
{
	temeraire::SSC32_Servo ssc32_servo;

	IKSolution = False;
	IKSolutionWarning = False;
	IKSolutionError = False;

	//Gait
	GaitSeq();
	/*
	   adapt_height();
	   if(sleeping == 0)
	   adapt_pitch_roll();
	 */
	//Balance calculations
	TotalTransX = 0; //reset values used for calculation of balance
	TotalTransZ = 0;
	TotalTransY = 0;
	TotalXBal = 0;
	TotalYBal = 0;
	TotalZBal = 0;

	if (BalanceMode>0) {
		BalCalcOneLeg(-RFPosX+BodyPosX+RFGaitPosX, RFPosZ+BodyPosZ+RFGaitPosZ,RFGaitPosY, RFOffsetX, RFOffsetZ);
		BalCalcOneLeg( -RMPosX+BodyPosX+RMGaitPosX, RMPosZ+BodyPosZ+RMGaitPosZ,RMGaitPosY, RMOffsetX, RMOffsetZ);
		BalCalcOneLeg( -RRPosX+BodyPosX+RRGaitPosX, RRPosZ+BodyPosZ+RRGaitPosZ,RRGaitPosY, RROffsetX, RROffsetZ);
		BalCalcOneLeg( LFPosX-BodyPosX+LFGaitPosX, LFPosZ+BodyPosZ+LFGaitPosZ,LFGaitPosY, LFOffsetX, LFOffsetZ);
		BalCalcOneLeg( LMPosX-BodyPosX+LMGaitPosX, LMPosZ+BodyPosZ+LMGaitPosZ,LMGaitPosY, LMOffsetX, LMOffsetZ);
		BalCalcOneLeg( LRPosX-BodyPosX+LRGaitPosX, LRPosZ+BodyPosZ+LRGaitPosZ,LRGaitPosY, LROffsetX, LROffsetZ);
		BalanceBody();
	}


	//Reset IKsolution indicators
	IKSolution = False;
	IKSolutionWarning = False;
	IKSolutionError = False;

	doBodyRot();

	do_IKs();



/*
	printf("%d %d %d / %d %d %d / %d %d %d / %d %d %d / %d %d %d / %d %d %d\n", (int)RFCoxaAngle, (int)RFFemurAngle, (int)RFTibiaAngle, (int)RMCoxaAngle, (int)RMFemurAngle, (int)RMTibiaAngle, (int)RRCoxaAngle, (int)RRFemurAngle, (int)RRTibiaAngle, (int)LFCoxaAngle, (int)LFFemurAngle, (int)LFTibiaAngle, (int)LMCoxaAngle, (int)LMFemurAngle, (int)LMTibiaAngle, (int)LRCoxaAngle, (int)LRFemurAngle, (int)LRTibiaAngle);
*/
	std::vector<double> vector_pose;
	
	vector_pose.push_back((double)-RRCoxaAngle );
	vector_pose.push_back((double)-RRFemurAngle );
	vector_pose.push_back((double)-RRTibiaAngle );
	vector_pose.push_back((double)10000 );

	vector_pose.push_back((double)-RMCoxaAngle );
	vector_pose.push_back((double)-RMFemurAngle );
	vector_pose.push_back((double)-RMTibiaAngle );
	vector_pose.push_back((double)10000 );

	vector_pose.push_back((double)-RFCoxaAngle);
	vector_pose.push_back((double)-RFFemurAngle);
	vector_pose.push_back((double)-RFTibiaAngle);
	vector_pose.push_back((double)10000 );

	vector_pose.push_back((double)10000 );
	vector_pose.push_back((double)10000 );
	vector_pose.push_back((double)10000 );
	vector_pose.push_back((double)10000 );



	vector_pose.push_back((double)LRCoxaAngle );
	vector_pose.push_back((double)LRFemurAngle );
	vector_pose.push_back((double)LRTibiaAngle );
	vector_pose.push_back((double)10000 );

	vector_pose.push_back((double)LMCoxaAngle );
	vector_pose.push_back((double)LMFemurAngle );
	vector_pose.push_back((double)LMTibiaAngle );
	vector_pose.push_back((double)10000 );

	vector_pose.push_back((double)LFCoxaAngle );
	vector_pose.push_back((double)LFFemurAngle );
	vector_pose.push_back((double)LFTibiaAngle );
	vector_pose.push_back((double)10000 );

	vector_pose.push_back((double)10000 );
	vector_pose.push_back((double)10000 );
	vector_pose.push_back((double)10000 );
	vector_pose.push_back((double)10000 );

	// ssc32_servo.position[0] = 10.0;
	ssc32_servo.position = vector_pose;

	servo_pub.publish(ssc32_servo);

}

int main(int argc, char **argv)
{


	ros::init(argc, argv, "Temeraire_Gait");
	TemeraireGait temeraire_gait;
	// Refresh rate
	ros::Rate loop_rate(4);                                // 35 with bluetooth


	while (ros::ok()) {
		ros::spinOnce();
		temeraire_gait.computeNextStep();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}

