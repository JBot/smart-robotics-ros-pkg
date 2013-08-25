#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "trajectory_msgs/JointTrajectory.h"

#include <sensor_msgs/JointState.h>
#include "dynamixel_msgs/JointState.h"
#include "dynamixel_controllers/SetSpeed.h"
#include "dynamixel_controllers/SetComplianceSlope.h"

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
// For gettimeofday
#include <getopt.h>
#include <sys/time.h>

#define STOCK_HEIGHT	80

#define min(x1,x2) ((x1) > (x2) ? (x2):(x1))
#define max(x1,x2) ((x1) > (x2) ? (x1):(x2))


class ARM_manager {
    public:
        ARM_manager();

        ros::Subscriber shoulder_sub_;
        ros::Subscriber elbow_sub_;
        ros::Subscriber ax12_sub_;
        ros::Subscriber joint_trajectory_sub_; // MoveIt
        
	ros::Publisher joint_pub;
	ros::Publisher shoulder_yaw_pub;
	ros::Publisher shoulder_pitch_pub;
	ros::Publisher elbow_pitch_pub;
	ros::Publisher wrist_pitch_pub;
	ros::Publisher wrist_yaw_pub;
	ros::Publisher wrist_roll_pub;
	ros::Publisher traj_feedback_pub; // MoveIt

        void periodic_function(void);

    private:
        void ax12Callback(const dynamixel_msgs::JointState::ConstPtr & pose);
        void shoulderCallback(const std_msgs::Float32::ConstPtr & msg);
        void elbowCallback(const std_msgs::Float32::ConstPtr & msg);
        void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr & msg); // MoveIt


        ros::NodeHandle nh;

	sensor_msgs::JointState joint_state;

};

ARM_manager::ARM_manager()
{
    /* SUBSCRIBERS */
    shoulder_sub_ = nh.subscribe < std_msgs::Float32 > ("shoulder_pitch_state", 5, &ARM_manager::shoulderCallback, this);
    elbow_sub_ = nh.subscribe < std_msgs::Float32 > ("elbow_pitch_state", 5, &ARM_manager::elbowCallback, this);
    ax12_sub_ = nh.subscribe < dynamixel_msgs::JointState > ("CONTROLLER_NAME/state", 5, &ARM_manager::ax12Callback, this);
    joint_trajectory_sub_ = nh.subscribe < trajectory_msgs::JointTrajectory > ("joint_path_command", 5, &ARM_manager::trajectoryCallback, this);
    

    /* PUBLISHERS */
    joint_pub = nh.advertise < sensor_msgs::JointState > ("arm/jointstate", 5);

    shoulder_yaw_pub = nh.advertise < std_msgs::Float64 > ("shoulder_yaw_joint", 5); 		// AX12
    shoulder_pitch_pub = nh.advertise < std_msgs::Float32 > ("shoulder_pitch_joint", 5);	// Motor
    elbow_pitch_pub = nh.advertise < std_msgs::Float32 > ("elbow_pitch_joint", 5);		// Motor
    wrist_pitch_pub = nh.advertise < std_msgs::Float64 > ("wrist_pitch_joint", 5);		// AX12
    wrist_yaw_pub = nh.advertise < std_msgs::Float64 > ("wrist_yaw_joint", 5);			// AX12
    wrist_roll_pub = nh.advertise < std_msgs::Float64 > ("wrist_roll_joint", 5);		// AX12
    traj_feedback_pub = nh.advertise < control_msgs::FollowJointTrajectoryFeedback > ("feedback_status", 5);		// MoveIt



    // AX12 joints
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.velocity.resize(6);
    joint_state.effort.resize(6);

    joint_state.name[0] ="shoulder_yaw_joint";
    joint_state.name[1] ="shoulder_pitch_joint";
    joint_state.name[2] ="elbow_pitch_joint";
    joint_state.name[3] ="wrist_pitch_joint";
    joint_state.name[4] ="wrist_yaw_joint";
    joint_state.name[5] ="wrist_roll_joint";

    joint_state.position[0] = 0.0;
    joint_state.position[1] = 0.0;
    joint_state.position[2] = 0.0;
    joint_state.position[3] = 0.0;
    joint_state.position[4] = 0.0;
    joint_state.position[5] = 0.0;

    joint_pub.publish(joint_state);


/*
    pose_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("object_pose", 5, &ARM_manager::poseCallback, this);

    left_done_sub_ = nh.subscribe < std_msgs::Int32 > ("left_arm_done", 5, &ARM_manager::leftdoneCallback, this);
    right_done_sub_ = nh.subscribe < std_msgs::Int32 > ("right_arm_done", 5, &ARM_manager::rightdoneCallback, this);

    left_pose_pub = nh.advertise < sensor_msgs::JointState > ("left_arm_pose", 5);
    right_pose_pub = nh.advertise < geometry_msgs::PoseStamped > ("right_arm_pose", 5);

    straight_move_pub = nh.advertise < std_msgs::Int32 > ("delta_ros", 3);

    pause_AI_pub = nh.advertise < std_msgs::Empty > ("pause_AI", 3);
    resume_AI_pub = nh.advertise < std_msgs::Empty > ("resume_AI", 3);

*/
}

void ARM_manager::ax12Callback(const dynamixel_msgs::JointState::ConstPtr & pose) {
	
	    // AX12 joints
    joint_state.header.stamp = ros::Time::now();

    joint_state.position[0] = pose->current_pos;
    joint_state.position[3] = 0.0;
    joint_state.position[4] = 0.0;
    joint_state.position[5] = 0.0;

//    joint_state.velocity = pose->velocity;

  //  joint_state.effort


    joint_pub.publish(joint_state);


}

void ARM_manager::shoulderCallback(const std_msgs::Float32::ConstPtr & msg) 
{
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[1] = msg->data;
    joint_pub.publish(joint_state);
}

void ARM_manager::elbowCallback(const std_msgs::Float32::ConstPtr & msg)
{
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[2] = msg->data;
    joint_pub.publish(joint_state);
}

void ARM_manager::trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr & msg) // MoveIt
{

}


void ARM_manager::periodic_function(void)
{
    joint_state.header.stamp = ros::Time::now();
    joint_pub.publish(joint_state);

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
    ros::init(argc, argv, "NESTOR_arm_manager");
    ARM_manager arm_manager;
    // Refresh rate
    ros::Rate loop_rate(10);                                // 35 with bluetooth
    while (ros::ok()) {
	//arm_manager.periodic_function();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}

