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

class ARM_manager {
    public:
        ARM_manager();

        ros::Subscriber action_sub_;
        ros::Subscriber firepose_sub_;
        ros::Subscriber fruitcolor_sub_;

	ros::Publisher alpha_pub;
	ros::Publisher delta_pub;
	ros::Publisher grip_pub;
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



	void joint_publish(void);


    private:
	void actionCallback(const std_msgs::Int32::ConstPtr & ptr);
	void fireposeCallback(const geometry_msgs::PoseStamped::ConstPtr & ptr);
	void fruitcolorCallback(const std_msgs::Int32::ConstPtr & ptr);

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

};

ARM_manager::ARM_manager()
{

  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  my_robot_model_loader = robot_model_loader;

  /* Get a shared pointer to the model */
  my_kinematic_model = my_robot_model_loader.getModel();

  /* Get and print the name of the coordinate frame in which the transforms for this model are computed*/
  ROS_INFO("Model frame: %s", my_kinematic_model->getModelFrame().c_str());

  /* WORKING WITH THE KINEMATIC STATE */
  /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(my_kinematic_model));
  my_kinematic_state = kinematic_state;

  /* Set all joints in this state to their default values */
  my_kinematic_state->setToDefaultValues();


  RA_joint_model_group = my_kinematic_model->getJointModelGroup("right_arm");
  RA_group_ = my_kinematic_state->getJointStateGroup("right_arm");

  /* Get the names of the joints in the right_arm*/
  RA_joint_names = RA_joint_model_group->getJointModelNames();

  LA_joint_model_group = my_kinematic_model->getJointModelGroup("left_arm");
  LA_group_ = my_kinematic_state->getJointStateGroup("left_arm");

  /* Get the names of the joints in the left_arm*/
  LA_joint_names = RA_joint_model_group->getJointModelNames();



  action_sub_ = nh.subscribe < std_msgs::Int32 > ("/ROBOT/action", 5, &ARM_manager::actionCallback, this);
  firepose_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("/ROBOT/firepose", 5, &ARM_manager::fireposeCallback, this); // inclure la couleur dans le twist
  fruitcolor_sub_ = nh.subscribe < std_msgs::Int32 > ("/ROBOT/fruitcolor", 5, &ARM_manager::fruitcolorCallback, this);

  alpha_pub = nh.advertise < std_msgs::Int32 > ("ROBOT/alpha_ros", 5);
  delta_pub = nh.advertise < std_msgs::Int32 > ("ROBOT/delta_ros", 5);

  Rjoint1_pub = nh.advertise < std_msgs::Float64 > ("Rarm1_joint", 5);
  Rjoint2_pub = nh.advertise < std_msgs::Float64 > ("Rarm2_joint", 5);
  Rjoint3_pub = nh.advertise < std_msgs::Float64 > ("Rarm3_joint", 5);
  Rjoint4_pub = nh.advertise < std_msgs::Float64 > ("Rarm4_joint", 5);
  Rjoint5_pub = nh.advertise < std_msgs::Float64 > ("Rarm5_joint", 5);

  Ljoint1_pub = nh.advertise < std_msgs::Float64 > ("Larm1_joint", 5);
  Ljoint2_pub = nh.advertise < std_msgs::Float64 > ("Larm2_joint", 5);
  Ljoint3_pub = nh.advertise < std_msgs::Float64 > ("Larm3_joint", 5);
  Ljoint4_pub = nh.advertise < std_msgs::Float64 > ("Larm4_joint", 5);
  Ljoint5_pub = nh.advertise < std_msgs::Float64 > ("Larm5_joint", 5);


  done_pub = nh.advertise < std_msgs::Empty > ("ROBOT/done", 5);


}

void ARM_manager::actionCallback(const std_msgs::Int32::ConstPtr & ptr)
{
}

void ARM_manager::fireposeCallback(const geometry_msgs::PoseStamped::ConstPtr & ptr)
{
}

void ARM_manager::fruitcolorCallback(const std_msgs::Int32::ConstPtr & ptr)
{
}

void ARM_manager::joint_publish(void)
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
  joint_values[3] = -1.0;
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
      ROS_INFO("Joint %s: %f", RA_joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
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
    ros::Rate loop_rate(10);                                // 35 with bluetooth
    while (ros::ok()) {
	arm_manager.joint_publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}




