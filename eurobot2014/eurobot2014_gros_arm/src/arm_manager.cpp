#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

#include "dynamixel_controllers/SetSpeed.h"
#include "dynamixel_controllers/SetComplianceSlope.h"
#include "dynamixel_msgs/JointState.h"
#include "dynamixel_msgs/MotorStateList.h"

#include <sensor_msgs/JointState.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/link_state.h>
#include <moveit/robot_state/joint_state.h>




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

#define min(x1,x2) ((x1) > (x2) ? (x2):(x1))
#define max(x1,x2) ((x1) > (x2) ? (x1):(x2))

class ARM_manager {
	public:
		ARM_manager();

		ros::Subscriber action_sub_;
		ros::Subscriber firepose_sub_;
		ros::Subscriber fruitcolor_sub_;
		ros::Subscriber start_game_sub_;

		ros::Subscriber Rdebug_sub_;
		ros::Subscriber Ldebug_sub_;


		ros::Publisher alpha_pub;
		ros::Publisher delta_pub;
		ros::Publisher grip_pub;
		ros::Publisher rpump_pub;
		ros::Publisher lpump_pub;
		ros::Publisher done_pub;

		ros::Publisher Rjoint1_pub;
		ros::Publisher Rjoint2_pub;
		ros::Publisher Rjoint3_pub;
		ros::Publisher Rjoint4_pub;
		ros::Publisher Rjoint5_pub;

		ros::Publisher Ljoint1_pub;
		ros::Publisher Ljoint2_pub;
		ros::Publisher Ljoint3_pub;
		ros::Publisher Ljoint4_pub;
		ros::Publisher Ljoint5_pub;

		ros::Publisher Rjoint_mini_pub;

		ros::Publisher Ljoint_mini_pub;


		ros::ServiceClient speed_Rjoint1;
		ros::ServiceClient speed_Rjoint2;
		ros::ServiceClient speed_Rjoint3;
		ros::ServiceClient speed_Rjoint4;
		ros::ServiceClient speed_Rjoint5;

		ros::ServiceClient slope_Rjoint1;
		ros::ServiceClient slope_Rjoint2;
		ros::ServiceClient slope_Rjoint3;
		ros::ServiceClient slope_Rjoint4;
		ros::ServiceClient slope_Rjoint5;

		ros::ServiceClient speed_Ljoint1;
		ros::ServiceClient speed_Ljoint2;
		ros::ServiceClient speed_Ljoint3;
		ros::ServiceClient speed_Ljoint4;
		ros::ServiceClient speed_Ljoint5;

		ros::ServiceClient slope_Ljoint1;
		ros::ServiceClient slope_Ljoint2;
		ros::ServiceClient slope_Ljoint3;
		ros::ServiceClient slope_Ljoint4;
		ros::ServiceClient slope_Ljoint5;






	private:
		void actionCallback(const std_msgs::Int32::ConstPtr & ptr);
		void fireposeCallback(const geometry_msgs::PoseStamped::ConstPtr & ptr);
		void fruitcolorCallback(const std_msgs::Int32::ConstPtr & ptr);
		void startgameCallback(const std_msgs::Empty::ConstPtr & ptr);
		void compute_RIK(geometry_msgs::PoseStamped pose);
		void compute_LIK(geometry_msgs::PoseStamped pose);

		void RdebugCallback(const geometry_msgs::PoseStamped::ConstPtr & ptr);
		void LdebugCallback(const geometry_msgs::PoseStamped::ConstPtr & ptr);

		void joint_publish(uint8_t type);
		void unstack_heart(uint8_t type);
		void swap_color(geometry_msgs::PoseStamped pose);
		void take_color(geometry_msgs::PoseStamped pose);
		void standard_pose(void);
		void init_pose(void);


		ros::NodeHandle nh;


		robot_model_loader::RobotModelLoader my_robot_model_loader;
		robot_model::RobotModelPtr my_kinematic_model;
		robot_state::RobotStatePtr my_kinematic_state;

		const robot_model::JointModelGroup* RA_joint_model_group;
		robot_state::JointStateGroup* RA_group_;
		std::vector<std::string> RA_joint_names;

		const robot_model::JointModelGroup* LA_joint_model_group;
		robot_state::JointStateGroup* LA_group_;
		std::vector<std::string> LA_joint_names;

		double prev_Rjoint1;
		double prev_Rjoint2;
		double prev_Rjoint3;
		double prev_Rjoint4;
		double prev_Rjoint5;

		double prev_Ljoint1;
		double prev_Ljoint2;
		double prev_Ljoint3;
		double prev_Ljoint4;
		double prev_Ljoint5;

		double max_speed;
};

ARM_manager::ARM_manager()
{

	// Load the robot model 
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	my_robot_model_loader = robot_model_loader;

	// Get a shared pointer to the model 
	my_kinematic_model = my_robot_model_loader.getModel();

	// Get and print the name of the coordinate frame in which the transforms for this model are computed
	ROS_INFO("Model frame: %s", my_kinematic_model->getModelFrame().c_str());

	// WORKING WITH THE KINEMATIC STATE 
	// Create a kinematic state - this represents the configuration for the robot represented by kinematic_model 
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(my_kinematic_model));
	my_kinematic_state = kinematic_state;

	// Set all joints in this state to their default values 
	my_kinematic_state->setToDefaultValues();


	RA_joint_model_group = my_kinematic_model->getJointModelGroup("right_arm");
	RA_group_ = my_kinematic_state->getJointStateGroup("right_arm");

	// Get the names of the joints in the right_arm
	RA_joint_names = RA_joint_model_group->getJointModelNames();

	LA_joint_model_group = my_kinematic_model->getJointModelGroup("left_arm");
	LA_group_ = my_kinematic_state->getJointStateGroup("left_arm");

	// Get the names of the joints in the left_arm
	LA_joint_names = LA_joint_model_group->getJointModelNames();



	action_sub_ = nh.subscribe < std_msgs::Int32 > ("/ROBOT/action", 5, &ARM_manager::actionCallback, this);
	firepose_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("/ROBOT/firepose", 5, &ARM_manager::fireposeCallback, this); // inclure la couleur dans le twist
	fruitcolor_sub_ = nh.subscribe < std_msgs::Int32 > ("/ROBOT/fruitcolor", 5, &ARM_manager::fruitcolorCallback, this);
	start_game_sub_ = nh.subscribe < std_msgs::Empty > ("/GENERAL/start_game", 5, &ARM_manager::startgameCallback, this);

	Rdebug_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("/DEBUG/right_arm_pose", 5, &ARM_manager::RdebugCallback, this); // inclure la couleur dans le twist
	Ldebug_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("/DEBUG/left_arm_pose", 5, &ARM_manager::LdebugCallback, this); // inclure la couleur dans le twist


	alpha_pub = nh.advertise < std_msgs::Int32 > ("/ROBOT/alpha_ros", 5);
	delta_pub = nh.advertise < std_msgs::Int32 > ("/ROBOT/delta_ros", 5);
	grip_pub = nh.advertise < std_msgs::Int8 > ("/ROBOT/grip", 5);

	rpump_pub = nh.advertise < std_msgs::Int8 > ("pump_ros", 5);
	lpump_pub = nh.advertise < std_msgs::Int8 > ("/ROBOT/lpump", 5);

	done_pub = nh.advertise < std_msgs::Empty > ("/ROBOT/done", 5);

	Rjoint1_pub = nh.advertise < std_msgs::Float64 > ("/Rlink1_controller/command", 5);
	Rjoint2_pub = nh.advertise < std_msgs::Float64 > ("/Rlink2_controller/command", 5);
	Rjoint3_pub = nh.advertise < std_msgs::Float64 > ("/Rlink3_controller/command", 5);
	Rjoint4_pub = nh.advertise < std_msgs::Float64 > ("/Rlink4_controller/command", 5);
	Rjoint5_pub = nh.advertise < std_msgs::Float64 > ("/Rlink5_controller/command", 5);

	Ljoint1_pub = nh.advertise < std_msgs::Float64 > ("/Llink1_controller/command", 5);
	Ljoint2_pub = nh.advertise < std_msgs::Float64 > ("/Llink2_controller/command", 5);
	Ljoint3_pub = nh.advertise < std_msgs::Float64 > ("/Llink3_controller/command", 5);
	Ljoint4_pub = nh.advertise < std_msgs::Float64 > ("/Llink4_controller/command", 5);
	Ljoint5_pub = nh.advertise < std_msgs::Float64 > ("/Llink5_controller/command", 5);

	speed_Rjoint1 = nh.serviceClient<dynamixel_controllers::SetSpeed>("/Rlink1_controller/set_speed", true);
	speed_Rjoint2 = nh.serviceClient<dynamixel_controllers::SetSpeed>("/Rlink2_controller/set_speed", true);
	speed_Rjoint3 = nh.serviceClient<dynamixel_controllers::SetSpeed>("/Rlink3_controller/set_speed", true);
	speed_Rjoint4 = nh.serviceClient<dynamixel_controllers::SetSpeed>("/Rlink4_controller/set_speed", true);
	speed_Rjoint5 = nh.serviceClient<dynamixel_controllers::SetSpeed>("/Rlink5_controller/set_speed", true);

	slope_Rjoint1 = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/Rlink1_controller/set_compliance_slope", true);
	slope_Rjoint2 = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/Rlink2_controller/set_compliance_slope", true);
	slope_Rjoint3 = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/Rlink3_controller/set_compliance_slope", true);
	slope_Rjoint4 = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/Rlink4_controller/set_compliance_slope", true);
	slope_Rjoint5 = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/Rlink5_controller/set_compliance_slope", true);

	speed_Ljoint1 = nh.serviceClient<dynamixel_controllers::SetSpeed>("/Llink1_controller/set_speed", true);
	speed_Ljoint2 = nh.serviceClient<dynamixel_controllers::SetSpeed>("/Llink2_controller/set_speed", true);
	speed_Ljoint3 = nh.serviceClient<dynamixel_controllers::SetSpeed>("/Llink3_controller/set_speed", true);
	speed_Ljoint4 = nh.serviceClient<dynamixel_controllers::SetSpeed>("/Llink4_controller/set_speed", true);
	speed_Ljoint5 = nh.serviceClient<dynamixel_controllers::SetSpeed>("/Llink5_controller/set_speed", true);

	slope_Ljoint1 = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/Llink1_controller/set_compliance_slope", true);
	slope_Ljoint2 = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/Llink2_controller/set_compliance_slope", true);
	slope_Ljoint3 = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/Llink3_controller/set_compliance_slope", true);
	slope_Ljoint4 = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/Llink4_controller/set_compliance_slope", true);
	slope_Ljoint5 = nh.serviceClient<dynamixel_controllers::SetComplianceSlope>("/Llink5_controller/set_compliance_slope", true);



	prev_Rjoint1 = 0.0;
	prev_Rjoint2 = 0.0;
	prev_Rjoint3 = 0.0;
	prev_Rjoint4 = 0.0;
	prev_Rjoint5 = 0.0;
	max_speed = 5.0;


	usleep(1000000);

	dynamixel_controllers::SetComplianceSlope tmp_slope;

	tmp_slope.request.slope = 90;
	if (slope_Rjoint1.call(tmp_slope))
	{
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service SetSlope");
	}
	if (slope_Rjoint2.call(tmp_slope))
	{
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service SetSlope");
	}
	if (slope_Rjoint3.call(tmp_slope))
	{
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service SetSlope");
	}
	if (slope_Rjoint4.call(tmp_slope))
	{
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service SetSlope");
	}
	if (slope_Rjoint5.call(tmp_slope))
	{
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service SetSlope");
	}
	if (slope_Ljoint1.call(tmp_slope))
	{
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service SetSlope");
	}
	if (slope_Ljoint2.call(tmp_slope))
	{
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service SetSlope");
	}
	if (slope_Ljoint3.call(tmp_slope))
	{
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service SetSlope");
	}
	if (slope_Ljoint4.call(tmp_slope))
	{
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service SetSlope");
	}
	if (slope_Ljoint5.call(tmp_slope))
	{
		//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
	}
	else
	{
		ROS_ERROR("Failed to call service SetSlope");
	}

	usleep(500000);

	//init_pose();
	standard_pose();
}

void ARM_manager::actionCallback(const std_msgs::Int32::ConstPtr & ptr)
{
}

void ARM_manager::fireposeCallback(const geometry_msgs::PoseStamped::ConstPtr & ptr)
{

	swap_color(*ptr);
	/*
	   geometry_msgs::PoseStamped tmp_pose;

	   tmp_pose.pose.position.x = 0.2;
	   tmp_pose.pose.position.y = 0.0;
	   tmp_pose.pose.position.z = 0.09;
	   tmp_pose.pose.orientation.x = 0.0;
	   tmp_pose.pose.orientation.y = 0.707;
	   tmp_pose.pose.orientation.z = 0.0;
	   tmp_pose.pose.orientation.w = 0.707;

	   compute_RIK(tmp_pose);

	   tmp_pose.pose.position.x = 0.27;
	   tmp_pose.pose.position.y = 0.09;
	   tmp_pose.pose.position.z = 0.25;
	   tmp_pose.pose.orientation.x = 0.0;
	   tmp_pose.pose.orientation.y = 0.707;
	   tmp_pose.pose.orientation.z = 0.0;
	   tmp_pose.pose.orientation.w = 0.707;

	   compute_LIK(tmp_pose);

	   joint_publish(2);

	   usleep(2000000);


	   tmp_pose.pose.position.x = 0.27;
	   tmp_pose.pose.position.y = -0.09;
	   tmp_pose.pose.position.z = 0.25;
	   tmp_pose.pose.orientation.x = 0.5;
	   tmp_pose.pose.orientation.y = 0.5;
	   tmp_pose.pose.orientation.z = 0.5;
	   tmp_pose.pose.orientation.w = 0.5;

	   compute_RIK(tmp_pose);

	   tmp_pose.pose.position.x = 0.27;
	   tmp_pose.pose.position.y = 0.09;
	   tmp_pose.pose.position.z = 0.25;
	   tmp_pose.pose.orientation.x = 0.5;
	   tmp_pose.pose.orientation.y = -0.5;
	   tmp_pose.pose.orientation.z = 0.5;
	   tmp_pose.pose.orientation.w = -0.5;

	   compute_LIK(tmp_pose);

	   joint_publish(2);

	   usleep(2000000);

	   tmp_pose.pose.position.x = 0.27;
	   tmp_pose.pose.position.y = -0.07;
	   tmp_pose.pose.position.z = 0.25;
	   tmp_pose.pose.orientation.x = 0.5;
	   tmp_pose.pose.orientation.y = 0.5;
	   tmp_pose.pose.orientation.z = 0.5;
	   tmp_pose.pose.orientation.w = 0.5;

	   compute_RIK(tmp_pose);

	   tmp_pose.pose.position.x = 0.27;
	   tmp_pose.pose.position.y = 0.07;
	   tmp_pose.pose.position.z = 0.25;
	   tmp_pose.pose.orientation.x = 0.5;
	   tmp_pose.pose.orientation.y = -0.5;
	   tmp_pose.pose.orientation.z = 0.5;
	   tmp_pose.pose.orientation.w = -0.5;

	   compute_LIK(tmp_pose);

	joint_publish(2);

	usleep(2000000);

	tmp_pose.pose.position.x = 0.27;
	tmp_pose.pose.position.y = -0.09;
	tmp_pose.pose.position.z = 0.25;
	tmp_pose.pose.orientation.x = 0.5;
	tmp_pose.pose.orientation.y = 0.5;
	tmp_pose.pose.orientation.z = 0.5;
	tmp_pose.pose.orientation.w = 0.5;

	compute_RIK(tmp_pose);

	tmp_pose.pose.position.x = 0.27;
	tmp_pose.pose.position.y = 0.09;
	tmp_pose.pose.position.z = 0.25;
	tmp_pose.pose.orientation.x = 0.5;
	tmp_pose.pose.orientation.y = -0.5;
	tmp_pose.pose.orientation.z = 0.5;
	tmp_pose.pose.orientation.w = -0.5;

	compute_LIK(tmp_pose);

	joint_publish(2);

	usleep(2000000);

	tmp_pose.pose.position.x = 0.25;
	tmp_pose.pose.position.y = -0.09;
	tmp_pose.pose.position.z = 0.22;
	tmp_pose.pose.orientation.x = 0.0;
	tmp_pose.pose.orientation.y = 0.707;
	tmp_pose.pose.orientation.z = 0.0;
	tmp_pose.pose.orientation.w = 0.707;

	compute_RIK(tmp_pose);

	tmp_pose.pose.position.x = 0.21;
	tmp_pose.pose.position.y = 0.02;
	tmp_pose.pose.position.z = 0.16;
	tmp_pose.pose.orientation.x = 0.0;
	tmp_pose.pose.orientation.y = 0.707;
	tmp_pose.pose.orientation.z = 0.0;
	tmp_pose.pose.orientation.w = 0.707;

	compute_LIK(tmp_pose);

	joint_publish(2);
	*/


		std_msgs::Empty done;
	done_pub.publish(done);

}

void ARM_manager::RdebugCallback(const geometry_msgs::PoseStamped::ConstPtr & ptr)
{
	compute_RIK(*ptr);
	joint_publish(0);
}

void ARM_manager::LdebugCallback(const geometry_msgs::PoseStamped::ConstPtr & ptr)
{
	compute_LIK(*ptr);
	joint_publish(1);
}



void ARM_manager::fruitcolorCallback(const std_msgs::Int32::ConstPtr & ptr)
{

	/* Get the names of the joints in the right_arm*/
	RA_joint_names = RA_joint_model_group->getJointModelNames();

	/* Get the joint positions for the right arm*/
	std::vector<double> joint_values;
	RA_group_->getVariableValues(joint_values);

	/* Print joint names and values */
	for(std::size_t i = 0; i < RA_joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", RA_joint_names[i].c_str(), joint_values[i]);
	}

	/* Set one joint in the right arm outside its joint limit */
	joint_values[0] = 0;
	joint_values[1] = 1.0;
	joint_values[2] = 0;
	joint_values[3] = 1.0;
	joint_values[4] = 1.0;
	RA_group_->setVariableValues(joint_values);



	/* Check whether any joint is outside its joint limits */
	ROS_INFO_STREAM("Current state is " << (RA_group_->satisfiesBounds() ? "valid" : "not valid"));

	/* Enforce the joint limits for this state and check again*/
	RA_group_->enforceBounds();
	ROS_INFO_STREAM("Current state is " << (RA_group_->satisfiesBounds() ? "valid" : "not valid"));

	RA_group_->getVariableValues(joint_values);

	/* Print joint names and values */
	for(std::size_t i = 0; i < RA_joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", RA_joint_names[i].c_str(), joint_values[i]);
	}

	/* FORWARD KINEMATICS */
	/* Compute FK for a set of random joint values*/
	//const Eigen::Affine3d &end_effector_state = link_state_->getGlobalLinkTransform();
	Eigen::Affine3d end_effector_state = RA_group_->getRobotState()->getLinkState("Rlink6")->getGlobalLinkTransform();

	RA_group_->getVariableValues(joint_values);

	/* Print joint names and values */
	for(std::size_t i = 0; i < RA_joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", RA_joint_names[i].c_str(), joint_values[i]);
	}

	ROS_INFO_STREAM("Current state is " << (RA_group_->satisfiesBounds() ? "valid" : "not valid"));


	/* Print end-effector pose. Remember that this is in the model frame */
	ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
	ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

	/* INVERSE KINEMATICS */
	/* Set joint state group to a set of random values*/
	RA_group_->setToRandomValues();

	RA_group_->getVariableValues(joint_values);

	/* Print joint names and values */
	for(std::size_t i = 0; i < RA_joint_names.size(); ++i)
	{
		ROS_INFO("Joint %s: %f", RA_joint_names[i].c_str(), joint_values[i]);
	}

	/* Do IK on the pose we just generated using forward kinematics
	 * Here 10 is the number of random restart and 0.1 is the allowed time after
	 * each restart
	 */
	bool found_ik = RA_group_->setFromIK(end_effector_state, 10, 0.1);

	/* Get and print the joint values */
	if (found_ik)
	{
		//kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		RA_group_->getVariableValues(joint_values);
		for(std::size_t i=0; i < RA_joint_names.size(); ++i)
		{
			//ROS_INFO("Joint %s: %f", RA_joint_names[i].c_str(), joint_values[i]);
		}
	}
	else
	{
		ROS_INFO("Did not find IK solution");
	}


}

void ARM_manager::startgameCallback(const std_msgs::Empty::ConstPtr & ptr)
{
        // Get the names of the joints in the right_arm
        RA_joint_names = RA_joint_model_group->getJointModelNames();
        LA_joint_names = LA_joint_model_group->getJointModelNames();

        // Get the joint positions for the right arm
        std::vector<double> joint_values;
        RA_group_->getVariableValues(joint_values);

        // Set one joint in the right arm outside its joint limit 
        joint_values[0] = 0;
        joint_values[1] = -0.25;
        joint_values[2] = 0;
        joint_values[3] = -2.3;
        joint_values[4] = 1.07;
        RA_group_->setVariableValues(joint_values);
        LA_group_->setVariableValues(joint_values);

        joint_publish(2);

        usleep(1500000);

	standard_pose();

}

void ARM_manager::compute_RIK(geometry_msgs::PoseStamped pose)
{
	/* Get the names of the joints in the right_arm*/
	RA_joint_names = RA_joint_model_group->getJointModelNames();

	/* Get the joint positions for the right arm*/
	std::vector<double> joint_values;
	RA_group_->getVariableValues(joint_values);



	/* Do IK on the pose we just generated using forward kinematics
	 * Here 10 is the number of random restart and 0.1 is the allowed time after
	 * each restart
	 */
	bool found_ik = RA_group_->setFromIK(pose.pose, "Rlink7", 10, 0.1);

	/* Get and print the joint values */
	if (found_ik)
	{
		//kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		RA_group_->getVariableValues(joint_values);
		for(std::size_t i=0; i < RA_joint_names.size(); ++i)
		{
			//ROS_INFO("RJoint %s: %f", RA_joint_names[i].c_str(), joint_values[i]);
		}
	}
	else
	{
		ROS_INFO("Did not find IK solution for right arm");
	}


}

void ARM_manager::compute_LIK(geometry_msgs::PoseStamped pose)
{
	/* Get the names of the joints in the right_arm*/
	LA_joint_names = LA_joint_model_group->getJointModelNames();

	/* Get the joint positions for the right arm*/
	std::vector<double> joint_values;
	LA_group_->getVariableValues(joint_values);



	/* Do IK on the pose we just generated using forward kinematics
	 * Here 10 is the number of random restart and 0.1 is the allowed time after
	 * each restart
	 */
	bool found_ik = LA_group_->setFromIK(pose.pose, "Llink7", 10, 0.1);

	/* Get and print the joint values */
	if (found_ik)
	{
		//kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		LA_group_->getVariableValues(joint_values);
		for(std::size_t i=0; i < LA_joint_names.size(); ++i)
		{
			//ROS_INFO("LJoint %s: %f", LA_joint_names[i].c_str(), joint_values[i]);
		}
	}
	else
	{
		ROS_INFO("Did not find IK solution for left arm");
	}
}

void ARM_manager::unstack_heart(uint8_t type)
{
	if(type == 0) { // Our heart

	}
	else { // Opponent heart

	}
}

void ARM_manager::swap_color(geometry_msgs::PoseStamped pose)
{
	geometry_msgs::PoseStamped tmp_pose;
	std_msgs::Int8 my_pump;
	ROS_INFO("SWAP");

	if(pose.pose.position.y > 0) { // left arm take the fire

		// Phase 1

		tmp_pose.pose.position.x = 0.27;
		tmp_pose.pose.position.y = -0.11;
		tmp_pose.pose.position.z = 0.25;
		tmp_pose.pose.orientation.x = 0.5;
		tmp_pose.pose.orientation.y = 0.5;
		tmp_pose.pose.orientation.z = 0.5;
		tmp_pose.pose.orientation.w = 0.5;

		compute_RIK(tmp_pose);

		tmp_pose.pose.position.x = pose.pose.position.x;
		tmp_pose.pose.position.y = pose.pose.position.y;
		tmp_pose.pose.position.z = pose.pose.position.z + 0.050;
		tmp_pose.pose.orientation.x = 0.0;
		tmp_pose.pose.orientation.y = 0.707;
		tmp_pose.pose.orientation.z = 0.0;
		tmp_pose.pose.orientation.w = 0.707;

		compute_LIK(tmp_pose);

		joint_publish(2);

		my_pump.data = 2;
		rpump_pub.publish(my_pump);

		usleep(1500000);

		// Phase 2

		tmp_pose.pose.position.x = 0.27;
		tmp_pose.pose.position.y = -0.1;
		tmp_pose.pose.position.z = 0.25;
		tmp_pose.pose.orientation.x = 0.5;
		tmp_pose.pose.orientation.y = 0.58;
		tmp_pose.pose.orientation.z = 0.5;
		tmp_pose.pose.orientation.w = 0.58;

		compute_RIK(tmp_pose);

		tmp_pose.pose.position.x = pose.pose.position.x;
		tmp_pose.pose.position.y = pose.pose.position.y;
		tmp_pose.pose.position.z = pose.pose.position.z;
		tmp_pose.pose.orientation.x = 0.0;
		tmp_pose.pose.orientation.y = 0.707;
		tmp_pose.pose.orientation.z = 0.0;
		tmp_pose.pose.orientation.w = 0.707;

		compute_LIK(tmp_pose);

		joint_publish(2);

		usleep(1000000);

		// Phase 3

		max_speed = 2.0;

		tmp_pose.pose.position.x = pose.pose.position.x;
		tmp_pose.pose.position.y = pose.pose.position.y;
		tmp_pose.pose.position.z = pose.pose.position.z + 0.050;
		tmp_pose.pose.orientation.x = 0.0;
		tmp_pose.pose.orientation.y = 0.707;
		tmp_pose.pose.orientation.z = 0.0;
		tmp_pose.pose.orientation.w = 0.707;

		compute_LIK(tmp_pose);

		joint_publish(1);

		usleep(1000000);

		// Phase 4

		tmp_pose.pose.position.x = 0.25;
		tmp_pose.pose.position.y = 0.1;
		tmp_pose.pose.position.z = 0.25;// + 0.02;
		tmp_pose.pose.orientation.x = 0.5;
		tmp_pose.pose.orientation.y = -0.58;
		tmp_pose.pose.orientation.z = 0.5;
		tmp_pose.pose.orientation.w = -0.58;

		compute_LIK(tmp_pose);

		joint_publish(1);

		my_pump.data = 3;
		rpump_pub.publish(my_pump);

		usleep(1500000);

		// Phase 5

		tmp_pose.pose.position.x = 0.27;
		tmp_pose.pose.position.y = -0.05;
		tmp_pose.pose.position.z = 0.25;
		tmp_pose.pose.orientation.x = 0.5;
		tmp_pose.pose.orientation.y = 0.58;
		tmp_pose.pose.orientation.z = 0.5;
		tmp_pose.pose.orientation.w = 0.58;

		compute_RIK(tmp_pose);

		tmp_pose.pose.position.x = 0.25;
		tmp_pose.pose.position.y = 0.05;
		tmp_pose.pose.position.z = 0.25;// + 0.02;
		tmp_pose.pose.orientation.x = 0.5;
		tmp_pose.pose.orientation.y = -0.58;
		tmp_pose.pose.orientation.z = 0.5;
		tmp_pose.pose.orientation.w = -0.58;

		compute_LIK(tmp_pose);

		joint_publish(2);

		//rpump_pub

		usleep(1500000);

		//lpump_pub
		my_pump.data = 1;
		rpump_pub.publish(my_pump);

		usleep(800000);

		// Phase 6

		tmp_pose.pose.position.x = 0.27;
		tmp_pose.pose.position.y = -0.1;
		tmp_pose.pose.position.z = 0.25;
		tmp_pose.pose.orientation.x = 0.5;
		tmp_pose.pose.orientation.y = 0.58;
		tmp_pose.pose.orientation.z = 0.5;
		tmp_pose.pose.orientation.w = 0.58;

		compute_RIK(tmp_pose);

		tmp_pose.pose.position.x = 0.27;
		tmp_pose.pose.position.y = 0.1;
		tmp_pose.pose.position.z = 0.25;
		tmp_pose.pose.orientation.x = 0.5;
		tmp_pose.pose.orientation.y = -0.58;
		tmp_pose.pose.orientation.z = 0.5;
		tmp_pose.pose.orientation.w = -0.58;

		compute_LIK(tmp_pose);

		joint_publish(2);

		usleep(1500000);

		// Phase 7

		tmp_pose.pose.position.x = pose.pose.position.x;
		if(pose.pose.position.y < 0.1)
			tmp_pose.pose.position.y = pose.pose.position.y;
		else
			tmp_pose.pose.position.y = 0.0;
		tmp_pose.pose.position.z = pose.pose.position.z + 0.050;
		tmp_pose.pose.orientation.x = 0.0;
		tmp_pose.pose.orientation.y = 0.707;
		tmp_pose.pose.orientation.z = 0.0;
		tmp_pose.pose.orientation.w = 0.707;

		compute_RIK(tmp_pose);

		tmp_pose.pose.position.x = 0.27;
		tmp_pose.pose.position.y = 0.11;
		tmp_pose.pose.position.z = 0.25;
		tmp_pose.pose.orientation.x = 0.0;
		tmp_pose.pose.orientation.y = 0.707;
		tmp_pose.pose.orientation.z = 0.0;
		tmp_pose.pose.orientation.w = 0.707;

		compute_LIK(tmp_pose);

		joint_publish(2);

		usleep(1500000);

		//rpump_pub
		my_pump.data = 0;
		rpump_pub.publish(my_pump);

		usleep(500000);

		max_speed = 5.0;

		standard_pose();


	}
	else { // right arm take the fire

	}

}

void ARM_manager::take_color(geometry_msgs::PoseStamped pose)
{

}

void ARM_manager::init_pose(void)
{
        /* Get the names of the joints in the right_arm*/
        RA_joint_names = RA_joint_model_group->getJointModelNames();
        LA_joint_names = LA_joint_model_group->getJointModelNames();

        /* Get the joint positions for the right arm*/
        std::vector<double> joint_values;
        RA_group_->getVariableValues(joint_values);

        /* Set one joint in the right arm outside its joint limit */
        joint_values[0] = 0;
        joint_values[1] = -0.25;
        joint_values[2] = 0;
        joint_values[3] = -2.3;
        joint_values[4] = 1.07;
        RA_group_->setVariableValues(joint_values);
        LA_group_->setVariableValues(joint_values);

	joint_publish(2);

	usleep(2500000);

        /* Set one joint in the right arm outside its joint limit */
        joint_values[0] = 0;
        joint_values[1] = -1.5;
        joint_values[2] = 0;
        joint_values[3] = -2.27;
        joint_values[4] = 1.76;
        RA_group_->setVariableValues(joint_values);
        LA_group_->setVariableValues(joint_values);

        joint_publish(2);

        usleep(2500000);


}

void ARM_manager::standard_pose(void)
{
	geometry_msgs::PoseStamped tmp_pose;


        tmp_pose.pose.position.x = 0.23;
        tmp_pose.pose.position.y = -0.06;
        tmp_pose.pose.position.z = 0.18;
        tmp_pose.pose.orientation.x = 0.0;
        tmp_pose.pose.orientation.y = 0.707;
        tmp_pose.pose.orientation.z = 0.0;
        tmp_pose.pose.orientation.w = 0.707;

        compute_RIK(tmp_pose);

        tmp_pose.pose.position.x = 0.23;
        tmp_pose.pose.position.y = 0.06;
        tmp_pose.pose.position.z = 0.18;
        tmp_pose.pose.orientation.x = 0.0;
        tmp_pose.pose.orientation.y = 0.707;
        tmp_pose.pose.orientation.z = 0.0;
        tmp_pose.pose.orientation.w = 0.707;

        compute_LIK(tmp_pose);

        joint_publish(2);

        usleep(500000);


	tmp_pose.pose.position.x = 0.20;
	tmp_pose.pose.position.y = -0.06;
	tmp_pose.pose.position.z = 0.24;
	tmp_pose.pose.orientation.x = 0.0;
	tmp_pose.pose.orientation.y = 0.707;
	tmp_pose.pose.orientation.z = 0.0;
	tmp_pose.pose.orientation.w = 0.707;

	compute_RIK(tmp_pose);

	tmp_pose.pose.position.x = 0.20;
	tmp_pose.pose.position.y = 0.06;
	tmp_pose.pose.position.z = 0.24;
	tmp_pose.pose.orientation.x = 0.0;
	tmp_pose.pose.orientation.y = 0.707;
	tmp_pose.pose.orientation.z = 0.0;
	tmp_pose.pose.orientation.w = 0.707;

	compute_LIK(tmp_pose);

	joint_publish(2);

	usleep(100000);

}

void ARM_manager::joint_publish(uint8_t type)
{

	if(type == 0) { // Only right ARM
		std::vector<double> joint_values;
		RA_group_->getVariableValues(joint_values);


		dynamixel_controllers::SetSpeed tmp_speed;

		double dist_Rjoint1 = fabs(prev_Rjoint1 - joint_values[0]);
		double dist_Rjoint2 = fabs(prev_Rjoint2 - joint_values[1]);
		double dist_Rjoint3 = fabs(prev_Rjoint3 - joint_values[2]);
		double dist_Rjoint4 = fabs(prev_Rjoint4 - joint_values[3]);
		double dist_Rjoint5 = fabs(prev_Rjoint5 - joint_values[4]);

		double distance_max = max(dist_Rjoint1, dist_Rjoint2);
		distance_max = max(distance_max, dist_Rjoint3);
		distance_max = max(distance_max, dist_Rjoint4);
		distance_max = max(distance_max, dist_Rjoint5);

		double speed_RJoint1 = max_speed * dist_Rjoint1 / distance_max;
		double speed_RJoint2 = max_speed * dist_Rjoint2 / distance_max;
		double speed_RJoint3 = max_speed * dist_Rjoint3 / distance_max;
		double speed_RJoint4 = max_speed * dist_Rjoint4 / distance_max;
		double speed_RJoint5 = max_speed * dist_Rjoint5 / distance_max;


		prev_Rjoint1 = joint_values[0];
		prev_Rjoint2 = joint_values[1];
		prev_Rjoint3 = joint_values[2];
		prev_Rjoint4 = joint_values[3];
		prev_Rjoint5 = joint_values[4];


		tmp_speed.request.speed = speed_RJoint1;
		if (speed_Rjoint1.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed1");
		}

		tmp_speed.request.speed = speed_RJoint2;
		if (speed_Rjoint2.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed2");
		}

		tmp_speed.request.speed = speed_RJoint3;
		if (speed_Rjoint3.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed3");
		}

		tmp_speed.request.speed = speed_RJoint4;
		if (speed_Rjoint4.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed4");
		}

		tmp_speed.request.speed = speed_RJoint5;
		if (speed_Rjoint5.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed4");
		}



		std_msgs::Float64 tmp;
		tmp.data = joint_values[0];
		Rjoint1_pub.publish(tmp);
		tmp.data = joint_values[1];
		Rjoint2_pub.publish(tmp);
		tmp.data = joint_values[2];
		Rjoint3_pub.publish(tmp);
		tmp.data = joint_values[3];
		Rjoint4_pub.publish(tmp);
		tmp.data = joint_values[4];
		Rjoint5_pub.publish(tmp);
	}
	else if(type == 1) { // Only left ARM
		std::vector<double> joint_values;
		LA_group_->getVariableValues(joint_values);

		dynamixel_controllers::SetSpeed tmp_speed;

		double dist_Ljoint1 = fabs(prev_Ljoint1 - joint_values[0]);
		double dist_Ljoint2 = fabs(prev_Ljoint2 - joint_values[1]);
		double dist_Ljoint3 = fabs(prev_Ljoint3 - joint_values[2]);
		double dist_Ljoint4 = fabs(prev_Ljoint4 - joint_values[3]);
		double dist_Ljoint5 = fabs(prev_Ljoint5 - joint_values[4]);

		double distance_max = max(dist_Ljoint1, dist_Ljoint2);
		distance_max = max(distance_max, dist_Ljoint3);
		distance_max = max(distance_max, dist_Ljoint4);
		distance_max = max(distance_max, dist_Ljoint5);

		double speed_LJoint1 = max_speed * dist_Ljoint1 / distance_max;
		double speed_LJoint2 = max_speed * dist_Ljoint2 / distance_max;
		double speed_LJoint3 = max_speed * dist_Ljoint3 / distance_max;
		double speed_LJoint4 = max_speed * dist_Ljoint4 / distance_max;
		double speed_LJoint5 = max_speed * dist_Ljoint5 / distance_max;


		prev_Ljoint1 = joint_values[0];
		prev_Ljoint2 = joint_values[1];
		prev_Ljoint3 = joint_values[2];
		prev_Ljoint4 = joint_values[3];
		prev_Ljoint5 = joint_values[4];


		tmp_speed.request.speed = speed_LJoint1;
		if (speed_Ljoint1.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed1");
		}

		tmp_speed.request.speed = speed_LJoint2;
		if (speed_Ljoint2.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed2");
		}

		tmp_speed.request.speed = speed_LJoint3;
		if (speed_Ljoint3.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed3");
		}

		tmp_speed.request.speed = speed_LJoint4;
		if (speed_Ljoint4.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed4");
		}

		tmp_speed.request.speed = speed_LJoint5;
		if (speed_Ljoint5.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed4");
		}


		std_msgs::Float64 tmp;
		tmp.data = joint_values[0];
		Ljoint1_pub.publish(tmp);
		tmp.data = joint_values[1];
		Ljoint2_pub.publish(tmp);
		tmp.data = joint_values[2];
		Ljoint3_pub.publish(tmp);
		tmp.data = joint_values[3];
		Ljoint4_pub.publish(tmp);
		tmp.data = joint_values[4];
		Ljoint5_pub.publish(tmp);
	}
	else if(type == 2) { // Both ARM

		std::vector<double> joint_valuesR;
		RA_group_->getVariableValues(joint_valuesR);
		std::vector<double> joint_valuesL;
		LA_group_->getVariableValues(joint_valuesL);


		dynamixel_controllers::SetSpeed tmp_speed;

		double dist_Rjoint1 = fabs(prev_Rjoint1 - joint_valuesR[0]);
		double dist_Rjoint2 = fabs(prev_Rjoint2 - joint_valuesR[1]);
		double dist_Rjoint3 = fabs(prev_Rjoint3 - joint_valuesR[2]);
		double dist_Rjoint4 = fabs(prev_Rjoint4 - joint_valuesR[3]);
		double dist_Rjoint5 = fabs(prev_Rjoint5 - joint_valuesR[4]);

		double dist_Ljoint1 = fabs(prev_Ljoint1 - joint_valuesL[0]);
		double dist_Ljoint2 = fabs(prev_Ljoint2 - joint_valuesL[1]);
		double dist_Ljoint3 = fabs(prev_Ljoint3 - joint_valuesL[2]);
		double dist_Ljoint4 = fabs(prev_Ljoint4 - joint_valuesL[3]);
		double dist_Ljoint5 = fabs(prev_Ljoint5 - joint_valuesL[4]);


		double distance_max = max(dist_Rjoint1, dist_Rjoint2);
		distance_max = max(distance_max, dist_Rjoint3);
		distance_max = max(distance_max, dist_Rjoint4);
		distance_max = max(distance_max, dist_Rjoint5);

		distance_max = max(distance_max, dist_Ljoint1);
		distance_max = max(distance_max, dist_Ljoint2);
		distance_max = max(distance_max, dist_Ljoint3);
		distance_max = max(distance_max, dist_Ljoint4);
		distance_max = max(distance_max, dist_Ljoint5);

		double speed_RJoint1 = max_speed * dist_Rjoint1 / distance_max;
		double speed_RJoint2 = max_speed * dist_Rjoint2 / distance_max;
		double speed_RJoint3 = max_speed * dist_Rjoint3 / distance_max;
		double speed_RJoint4 = max_speed * dist_Rjoint4 / distance_max;
		double speed_RJoint5 = max_speed * dist_Rjoint5 / distance_max;

		double speed_LJoint1 = max_speed * dist_Ljoint1 / distance_max;
		double speed_LJoint2 = max_speed * dist_Ljoint2 / distance_max;
		double speed_LJoint3 = max_speed * dist_Ljoint3 / distance_max;
		double speed_LJoint4 = max_speed * dist_Ljoint4 / distance_max;
		double speed_LJoint5 = max_speed * dist_Ljoint5 / distance_max;

		prev_Rjoint1 = joint_valuesR[0];
		prev_Rjoint2 = joint_valuesR[1];
		prev_Rjoint3 = joint_valuesR[2];
		prev_Rjoint4 = joint_valuesR[3];
		prev_Rjoint5 = joint_valuesR[4];

		prev_Ljoint1 = joint_valuesL[0];
		prev_Ljoint2 = joint_valuesL[1];
		prev_Ljoint3 = joint_valuesL[2];
		prev_Ljoint4 = joint_valuesL[3];
		prev_Ljoint5 = joint_valuesL[4];


		tmp_speed.request.speed = speed_RJoint1;
		if (speed_Rjoint1.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed1");
		}

		tmp_speed.request.speed = speed_RJoint2;
		if (speed_Rjoint2.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed2");
		}

		tmp_speed.request.speed = speed_RJoint3;
		if (speed_Rjoint3.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed3");
		}

		tmp_speed.request.speed = speed_RJoint4;
		if (speed_Rjoint4.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed4");
		}

		tmp_speed.request.speed = speed_RJoint5;
		if (speed_Rjoint5.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed4");
		}



		tmp_speed.request.speed = speed_LJoint1;
		if (speed_Ljoint1.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed1");
		}

		tmp_speed.request.speed = speed_LJoint2;
		if (speed_Ljoint2.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed2");
		}

		tmp_speed.request.speed = speed_LJoint3;
		if (speed_Ljoint3.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed3");
		}

		tmp_speed.request.speed = speed_LJoint4;
		if (speed_Ljoint4.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed4");
		}

		tmp_speed.request.speed = speed_LJoint5;
		if (speed_Ljoint5.call(tmp_speed))
		{
			//ROS_INFO("Sum: %ld", (long int)srv.response.sum);
		}
		else
		{
			ROS_ERROR("Failed to call service SetSpeed4");
		}


		std_msgs::Float64 tmp;
		tmp.data = joint_valuesR[0];
		Rjoint1_pub.publish(tmp);
		tmp.data = joint_valuesR[1];
		Rjoint2_pub.publish(tmp);
		tmp.data = joint_valuesR[2];
		Rjoint3_pub.publish(tmp);
		tmp.data = joint_valuesR[3];
		Rjoint4_pub.publish(tmp);
		tmp.data = joint_valuesR[4];
		Rjoint5_pub.publish(tmp);

		tmp.data = joint_valuesL[0];
		Ljoint1_pub.publish(tmp);
		tmp.data = joint_valuesL[1];
		Ljoint2_pub.publish(tmp);
		tmp.data = joint_valuesL[2];
		Ljoint3_pub.publish(tmp);
		tmp.data = joint_valuesL[3];
		Ljoint4_pub.publish(tmp);
		tmp.data = joint_valuesL[4];
		Ljoint5_pub.publish(tmp);




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
	ros::init(argc, argv, "ARM_manager");
	ARM_manager arm_manager;
	// Refresh rate
	ros::Rate loop_rate(20);                                // 35 with bluetooth

	while (ros::ok()) {
		//arm_manager.joint_publish(0);
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::Duration(2.0).sleep();

	ros::shutdown();
}




