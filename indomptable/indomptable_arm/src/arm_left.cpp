#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include "dynamixel_controllers/SetSpeed.h"
#include "dynamixel_controllers/SetComplianceSlope.h"
#include "dynamixel_msgs/JointState.h"

#include <sensor_msgs/JointState.h>

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
//#include <math.h>
//#include <algorithm>

#include <vector>



#include <unistd.h>
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

/*
#define False 0
#define True  1 
#define FALSE 0
#define TRUE  1
 */
#define SLEEP_COEFF 2000 //1600 //1000 //2000

#define CoxaLength 78 //43      //Length of the Coxa [mm]
#define FemurLength 82 //76      //Length of the Femur [mm]
#define TibiaLength 94 //66     //NEW Lenght of the Tibia [mm]
//#define CoxaAngle 0      //Default Coxa setup angle

#define STOCK_HEIGHT	80

#define MAX_SPEED 3.0 //5.0


#define min(x1,x2) ((x1) > (x2) ? (x2):(x1))
#define max(x1,x2) ((x1) > (x2) ? (x1):(x2))


class indomptableARM {
    public:
        indomptableARM();

        ros::Subscriber pose_sub_;
        ros::Subscriber pump_sub_;
        ros::Publisher coxa_pub;
        ros::Publisher femur_pub;
        ros::Publisher tibia_pub;
        ros::Publisher ankle_pub;
        ros::Publisher pump_pub;

        ros::ServiceClient client_shoulder_roll;
        ros::ServiceClient client_shoulder_lift;
        ros::ServiceClient client_elbow;
        ros::ServiceClient client_wrist;

        ros::ServiceClient slope_shoulder_roll;
        ros::ServiceClient slope_shoulder_lift;
        ros::ServiceClient slope_elbow;
        ros::ServiceClient slope_wrist;

        ros::Subscriber sub_shoulder_roll;
        ros::Subscriber sub_shoulder_lift;
        ros::Subscriber sub_elbow;
        ros::Subscriber sub_wrist;

        ros::Publisher joint_pub;

        ros::Publisher done_pub;

        void joint_publish(void);

    private:
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
        void pumpCallback(const std_msgs::Int32::ConstPtr & pumpfeedback);
        void LegIK(signed int IKFeetPosX, signed int IKFeetPosY, signed int IKFeetPosZ);
        void ServoDriver();
        void FreeServos();
        void takeCDinTotem(signed int height);
        void takeBARinTotem(void);
        void takeCDGround(signed int x, signed int y);
        void takeBARGround(signed int x, signed int y);
        void waitState(void);
        void releaseObject(void);
        void releaseBAR(void);
        void waitMoveEnd(void);

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
        double prev_CoxaAngle;   //Actual Angle of the Right Front Leg
        double prev_FemurAngle;
        double prev_TibiaAngle;
        double prev_AnkleAngle;
        double prev_RollAngle;
        double prev_HandAngle;
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

        sensor_msgs::JointState joint_state;

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

    usleep(1000000);

    tmp_string = input_name;
    tmp_string.append("_shoulder_roll_joint");
    coxa_pub = nh.advertise < std_msgs::Float64 > ("shoulder_roll_joint", 5);
    tmp_string = input_name;
    tmp_string.append("_shoulder_lift_joint");
    femur_pub = nh.advertise < std_msgs::Float64 > ("shoulder_lift_joint", 5);
    tmp_string = input_name;
    tmp_string.append("_elbow_joint");
    tibia_pub = nh.advertise < std_msgs::Float64 > ("elbow_joint", 5);
    tmp_string = input_name;
    tmp_string.append("_wrist_joint");
    ankle_pub = nh.advertise < std_msgs::Float64 > ("wrist_joint", 5);

    tmp_string = input_name;
    tmp_string.append("_pump");
    pump_pub = nh.advertise < std_msgs::Int32 > ("pump", 5);
    tmp_string = input_name;
    tmp_string.append("_pump_feedback");
    pump_sub_ = nh.subscribe < std_msgs::Int32 > ("pump_feedback", 5, &indomptableARM::pumpCallback, this);

    usleep(1000000);

    client_shoulder_roll = nh.serviceClient<dynamixel_controllers::SetSpeed>("/left_shoulder_roll_controller/set_speed", true);
    client_shoulder_lift = nh.serviceClient<dynamixel_controllers::SetSpeed>("/left_shoulder_lift_controller/set_speed", true);
    client_elbow = nh.serviceClient<dynamixel_controllers::SetSpeed>("/left_elbow_controller/set_speed", true);
    client_wrist = nh.serviceClient<dynamixel_controllers::SetSpeed>("/left_wrist_controller/set_speed", true);

    slope_shoulder_roll = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/left_shoulder_roll_controller/set_compliance_slope", true);
    slope_shoulder_lift = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/left_shoulder_lift_controller/set_compliance_slope", true);
    slope_elbow = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/left_elbow_controller/set_compliance_slope", true);
    slope_wrist = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/left_wrist_controller/set_compliance_slope", true);

    joint_pub = nh.advertise < sensor_msgs::JointState > ("joint_states", 1);
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(4);
    joint_state.position.resize(4);
    joint_state.velocity.resize(4);
    joint_state.effort.resize(4);

    joint_state.name[0] ="left_shoulder_roll_joint";
    joint_state.name[1] ="left_shoulder_lift_joint";
    joint_state.name[2] ="left_elbow_joint";
    joint_state.name[3] ="left_wrist_joint";

    joint_state.position[0] = 0.0;
    joint_state.position[1] = 0.0;
    joint_state.position[2] = 0.0;
    joint_state.position[3] = 0.0;

    joint_pub.publish(joint_state);

    done_pub = nh.advertise < std_msgs::Int32 > ("left_arm_done", 2);


    prev_CoxaAngle = 0;
    prev_FemurAngle = 0;
    prev_TibiaAngle = 0;
    prev_AnkleAngle = 0;
    prev_RollAngle = 0;
    prev_HandAngle = 0;


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

    usleep(1000000);

    dynamixel_controllers::SetComplianceSlope tmp_slope;

    tmp_slope.request.slope = 90;
    if (slope_shoulder_roll.call(tmp_slope))
    {
        //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service SetSlope");
    }
    if (slope_shoulder_lift.call(tmp_slope))
    {
        //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service SetSlope");
    }
    if (slope_elbow.call(tmp_slope))
    {
        //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service SetSlope");
    }
    if (slope_wrist.call(tmp_slope))
    {
        //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service SetSlope");
    }


    joint_publish();

    /*
     */

    waitState();

}

void indomptableARM::joint_publish(void)
{

    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(4);
    joint_state.position.resize(4);
    joint_state.velocity.resize(4);
    joint_state.effort.resize(4);

    joint_state.name[0] ="left_shoulder_roll_joint";
    joint_state.name[1] ="left_shoulder_lift_joint";
    joint_state.name[2] ="left_elbow_joint";
    joint_state.name[3] ="left_wrist_joint";

    joint_state.position[0] = prev_CoxaAngle;
    joint_state.position[1] = -prev_FemurAngle;
    joint_state.position[2] = prev_TibiaAngle;
    joint_state.position[3] = -prev_AnkleAngle;

    joint_pub.publish(joint_state);
}

void indomptableARM::poseCallback(const geometry_msgs::PoseStamped::ConstPtr & pose)
{

    //usleep(5000000); // For debug only

    if( (pose->pose.position.z < 0.075) && (pose->pose.position.z > 0.070) ) {
        takeCDGround((int)(pose->pose.position.x * 1000), (int)(pose->pose.position.y * 1000));
    }
    if( (pose->pose.position.z < 0.065) && (pose->pose.position.z > 0.060) ) {
        takeBARGround((int)(pose->pose.position.x * 1000), (int)(pose->pose.position.y * 1000));
    }
    if( (pose->pose.position.z < 0.002) && (pose->pose.position.z > -0.002) ) {
        takeBARinTotem();
    }
    if( (pose->pose.position.z < -0.070) && (pose->pose.position.z > -0.075) ) {
        takeCDinTotem((int)(pose->pose.position.z * 1000));
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

    //IKFeetPosX = IKFeetPosX - 

    //Length between the Coxa and Feet
    IKFeetPosXZ =  sqrt((double)((IKFeetPosX*IKFeetPosX)+(IKFeetPosZ*IKFeetPosZ)));

    //IKSW - Length between shoulder and wrist
    IKSW = sqrt((double)(((IKFeetPosXZ-CoxaLength)*(IKFeetPosXZ-CoxaLength))+(IKFeetPosY*IKFeetPosY)));

    //IKA1 - Angle between SW line and the ground in rad
    IKA1 = atan2(IKFeetPosXZ-CoxaLength, IKFeetPosY);

    //IKA2 - ?
    IKA2 = acos( ((double)((FemurLength*FemurLength) - (TibiaLength*TibiaLength)) + (IKSW*IKSW)) / ((double)(2*FemurLength) * IKSW) );

    //IKFemurAngle
    IKFemurAngle = ( -(IKA1 + IKA2) )+1.570796;

    //IKTibiaAngle
    IKTibiaAngle = -( 1.570796-(acos( ( (double)(FemurLength*FemurLength) + (TibiaLength*TibiaLength) - (IKSW*IKSW) ) / ((double)(2*FemurLength*TibiaLength)) )) );

    //IKCoxaAngle
    IKCoxaAngle = ((atan2(IKFeetPosZ, IKFeetPosX)));


    return;
}


void indomptableARM::takeCDinTotem(signed int height){

    /*	LegIK((int)(180), (int)(-(42)), (int)(0));
        DesAnkleAngle = -1.570796;
        CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
        FemurAngle = -IKFemurAngle;
        TibiaAngle = (1.570796 - IKTibiaAngle);
        AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
        RollAngle = -IKCoxaAngle;
        ActualGaitSpeed = 300;
        ServoDriver();

        waitMoveEnd();
     */
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


    LegIK((int)(230), (int)(-(42)), (int)(0));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();
    //usleep((ActualGaitSpeed+50+50)*SLEEP_COEFF);

    LegIK((int)(230), (int)((12)), (int)(0));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();
    //usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

    LegIK((int)(180), (int)(-(42)), (int)(0));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    //usleep((ActualGaitSpeed+50)*SLEEP_COEFF);
    waitMoveEnd();

    releaseObject();

    waitState();

}

void indomptableARM::takeBARinTotem(void){

    LegIK((int)(130), (int)(120), (int)(0));
    DesAnkleAngle = 0;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();

    LegIK((int)(150), (int)(120), (int)(0));
    DesAnkleAngle = 0;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();

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



    LegIK((int)(170), (int)(120), (int)(0));
    DesAnkleAngle = 0;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();

    LegIK((int)(130), (int)(120), (int)(0));
    DesAnkleAngle = 0;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();



    releaseObject();

    waitState();

}

void indomptableARM::takeCDGround(signed int x, signed int y){

    /*        LegIK((int)(80), (int)(100), (int)(y));
              DesAnkleAngle = -1.570796;
              CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
              FemurAngle = -IKFemurAngle;
              TibiaAngle = (1.570796 - IKTibiaAngle);
              AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
              RollAngle = -IKCoxaAngle;
              ActualGaitSpeed = 300;
              ServoDriver();

              usleep((ActualGaitSpeed+50)*SLEEP_COEFF);
     */
    // (80)
    LegIK((int)(x), (int)(135), (int)(y));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    usleep((ActualGaitSpeed)*SLEEP_COEFF);

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

    LegIK((int)(x), (int)(150), (int)(y));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    usleep((ActualGaitSpeed)*SLEEP_COEFF);


    LegIK((int)(x), (int)(170), (int)(y));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

    LegIK((int)(x+10), (int)(100), (int)(y));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    usleep((ActualGaitSpeed+50)*SLEEP_COEFF);


    releaseObject();

    waitState();
}

void indomptableARM::takeBARGround(signed int x, signed int y){

    // (80)
    LegIK((int)(x), (int)(100), (int)(y));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    usleep((ActualGaitSpeed)*SLEEP_COEFF);

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

    LegIK((int)(x), (int)(120), (int)(y));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    usleep((ActualGaitSpeed)*SLEEP_COEFF);

/*
    LegIK((int)(x), (int)(170), (int)(y));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    usleep((ActualGaitSpeed+50)*SLEEP_COEFF);
*/
    LegIK((int)(x+10), (int)(90), (int)(y));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    usleep((ActualGaitSpeed+50)*SLEEP_COEFF);


    releaseBAR();

    waitState();
}



void indomptableARM::waitMoveEnd(void){
    /*
       usleep(50000); // For debug only
       ros::spinOnce();
       usleep(50000); // For debug only
       ros::spinOnce();
       while(wrist_state.is_moving == true) {

       ROS_INFO("Bla : %d", wrist_state.is_moving);
       usleep(50000); // For debug only
       ros::spinOnce();

       }

       ROS_INFO("Bla : %d", wrist_state.is_moving);
     */
    usleep((200+50)*SLEEP_COEFF);


}

void indomptableARM::waitState(void){

    LegIK((int)(70), (int)(70), (int)(0));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();

    std_msgs::Int32 tmp_done;
    tmp_done.data = 1;
    done_pub.publish(tmp_done);

    //usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

}

void indomptableARM::releaseObject(void){

    LegIK((int)(150), (int)((0)), (int)(0));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();
    //usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

    LegIK((int)(160), (int)(-(10)), (int)(0));
    DesAnkleAngle = -2.37;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();

    LegIK((int)(100), (int)((30)), (int)(0));
    DesAnkleAngle = -2.7;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();

    LegIK((int)(50), (int)((40)), (int)(0));
    DesAnkleAngle = -2.9;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();
    /*
       LegIK((int)(0), (int)((50)), (int)(0));
       DesAnkleAngle = -2.9;
       CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
       FemurAngle = -IKFemurAngle;
       TibiaAngle = (1.570796 - IKTibiaAngle);
       AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
       RollAngle = -IKCoxaAngle;
       ActualGaitSpeed = 300;
       ServoDriver();

       waitMoveEnd();
     */


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

    waitMoveEnd();
    //usleep((ActualGaitSpeed)*SLEEP_COEFF);

    LegIK((int)(150), (int)((0)), (int)(0));
    DesAnkleAngle = -2.7;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();


}

void indomptableARM::releaseBAR(void){

    LegIK((int)(150), (int)((0)), (int)(0));
    DesAnkleAngle = -1.570796;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();
    //usleep((ActualGaitSpeed+50)*SLEEP_COEFF);

    LegIK((int)(160), (int)((10)), (int)(0));
    DesAnkleAngle = -2.7;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();

    LegIK((int)(100), (int)((30)), (int)(0));
    DesAnkleAngle = -2.9;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();

    LegIK((int)(50), (int)((30)), (int)(0));
    DesAnkleAngle = -3.1;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();
    /*
       LegIK((int)(0), (int)((50)), (int)(0));
       DesAnkleAngle = -2.9;
       CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
       FemurAngle = -IKFemurAngle;
       TibiaAngle = (1.570796 - IKTibiaAngle);
       AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
       RollAngle = -IKCoxaAngle;
       ActualGaitSpeed = 300;
       ServoDriver();

       waitMoveEnd();
     */


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

    waitMoveEnd();
    //usleep((ActualGaitSpeed)*SLEEP_COEFF);

    LegIK((int)(150), (int)((0)), (int)(0));
    DesAnkleAngle = -2.7;
    CoxaAngle  = IKCoxaAngle ; //Angle for the basic setup for the front leg   
    FemurAngle = -IKFemurAngle;
    TibiaAngle = (1.570796 - IKTibiaAngle);
    AnkleAngle = -FemurAngle + TibiaAngle + DesAnkleAngle;
    RollAngle = -IKCoxaAngle;
    ActualGaitSpeed = 300;
    ServoDriver();

    waitMoveEnd();


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

    printf("%f %f %f %f\n", CoxaAngle, FemurAngle, TibiaAngle, AnkleAngle);

    std_msgs::Float64 tmp;
    dynamixel_controllers::SetSpeed tmp_speed;

    double dist_CoxaAngle = fabs(prev_CoxaAngle - CoxaAngle);
    double dist_FemurAngle = fabs(prev_FemurAngle - FemurAngle);
    double dist_TibiaAngle = fabs(prev_TibiaAngle - TibiaAngle);
    double dist_AnkleAngle = fabs(prev_AnkleAngle - AnkleAngle);
    double dist_RollAngle = fabs(prev_RollAngle - RollAngle);
    double dist_HandAngle = fabs(prev_HandAngle - HandAngle);

    double distance_max = max(dist_CoxaAngle, dist_FemurAngle);
    distance_max = max(distance_max, dist_TibiaAngle);
    distance_max = max(distance_max, dist_AnkleAngle);
    distance_max = max(distance_max, dist_RollAngle);
    distance_max = max(distance_max, dist_HandAngle);

    double speed_CoxaAngle = MAX_SPEED * dist_CoxaAngle / distance_max;
    double speed_FemurAngle = MAX_SPEED * dist_FemurAngle / distance_max;
    double speed_TibiaAngle = MAX_SPEED * dist_TibiaAngle / distance_max;
    double speed_AnkleAngle = MAX_SPEED * dist_AnkleAngle / distance_max;
    double speed_RollAngle = MAX_SPEED * dist_RollAngle / distance_max;


    prev_CoxaAngle = CoxaAngle;
    prev_FemurAngle = FemurAngle;
    prev_TibiaAngle = TibiaAngle;
    prev_AnkleAngle = AnkleAngle;
    prev_RollAngle = RollAngle;
    prev_HandAngle = HandAngle;

    printf("speed %f %f %f %f\n", speed_CoxaAngle, speed_FemurAngle, speed_TibiaAngle, speed_AnkleAngle);

    tmp_speed.request.speed = speed_CoxaAngle;
    if (client_shoulder_roll.call(tmp_speed))
    {
        //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service SetSpeed1");
    }

    tmp_speed.request.speed = speed_FemurAngle;
    if (client_shoulder_lift.call(tmp_speed))
    {
        //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service SetSpeed2");
    }

    tmp_speed.request.speed = speed_TibiaAngle;
    if (client_elbow.call(tmp_speed))
    {
        //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service SetSpeed3");
    }

    tmp_speed.request.speed = speed_AnkleAngle;
    if (client_wrist.call(tmp_speed))
    {
        //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service SetSpeed4");
    }


    joint_publish();



    tmp.data = -CoxaAngle;
    coxa_pub.publish(tmp);
    tmp.data = FemurAngle;
    femur_pub.publish(tmp);
    tmp.data = TibiaAngle;
    tibia_pub.publish(tmp);
    tmp.data = -AnkleAngle;
    ankle_pub.publish(tmp);

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
    ros::Rate loop_rate(10);                                // 35 with bluetooth
    while (ros::ok()) {
        indomptablearm.joint_publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}

