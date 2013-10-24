/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2012, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Sachin Chitta */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/link_state.h>
#include <moveit/robot_state/joint_state.h>
//#include <moveit/robot_state/joint_state_group.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  /* Get a shared pointer to the model */
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  /* Get and print the name of the coordinate frame in which the transforms for this model are computed*/
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  /* WORKING WITH THE KINEMATIC STATE */
  /* Create a kinematic state - this represents the configuration for the robot represented by kinematic_model */
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  /* Set all joints in this state to their default values */
  kinematic_state->setToDefaultValues();

  const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
  robot_state::JointStateGroup* group_ = kinematic_state->getJointStateGroup("right_arm");

  /* Get the names of the joints in the right_arm*/
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

  /* Get the joint positions for the right arm*/
  std::vector<double> joint_values;
  group_->getVariableValues(joint_values);

  /* Print joint names and values */
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  /* Set one joint in the right arm outside its joint limit */
  joint_values[0] = 0;
  joint_values[1] = 1.0;
  joint_values[2] = 0;
  joint_values[3] = -1.0;
  group_->setVariableValues(joint_values); 

  /* Check whether any joint is outside its joint limits */
  ROS_INFO_STREAM("Current state is " << (group_->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  group_->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (group_->satisfiesBounds() ? "valid" : "not valid"));

  group_->getVariableValues(joint_values);

  /* Print joint names and values */
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }


  /* FORWARD KINEMATICS */
  /* Compute FK for a set of random joint values*/
  //const Eigen::Affine3d &end_effector_state = link_state_->getGlobalLinkTransform();
  Eigen::Affine3d end_effector_state = group_->getRobotState()->getLinkState("Rlink6")->getGlobalLinkTransform();
  ROS_INFO("Sauvegarde etat end effector");


  group_->getVariableValues(joint_values);

  /* Print joint names and values */
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  ROS_INFO_STREAM("Current state is " << (group_->satisfiesBounds() ? "valid" : "not valid"));


  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  /* INVERSE KINEMATICS */
  /* Set joint state group to a set of random values*/
  group_->setToRandomValues();

  group_->getVariableValues(joint_values);

  /* Print joint names and values */
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }


  /* Do IK on the pose we just generated using forward kinematics
* Here 10 is the number of random restart and 0.1 is the allowed time after
* each restart
*/
  bool found_ik = group_->setFromIK(end_effector_state, 10, 0.1);

    ROS_INFO("Aprés calcul de IK");

  /* Get and print the joint values */
  if (found_ik)
  {
    //kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    group_->getVariableValues(joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  /* DIFFERENTIAL KINEMATICS */
  /* Get and print the Jacobian for the right arm*/
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;
  group_->getJacobian("Rlink6",
                               reference_point_position,
                               jacobian);
  ROS_INFO_STREAM("Jacobian: " << jacobian);

  ros::shutdown();
  return 0;
}
