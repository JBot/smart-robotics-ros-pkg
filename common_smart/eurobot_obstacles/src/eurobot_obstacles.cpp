#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "my_obstacles");

  ros::NodeHandle nh;

  ros::Publisher object_in_map_pub_;
  object_in_map_pub_  = nh.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);

  ros::Duration(2.0).sleep();
  {
  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "tree";
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "map";
  cylinder_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::CYLINDER;
  object.dimensions.resize(2);
  object.dimensions[0] = .04;
  object.dimensions[1] = .25;
  geometry_msgs::Pose pose;
  pose.position.x = 0;
  pose.position.y = 1.0;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  cylinder_object.id = "tree";
  object_in_map_pub_.publish(cylinder_object);
  }

  {
  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "blue_totem_low";
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "map";
  cylinder_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = .25;
  object.dimensions[1] = .25;
  object.dimensions[2] = .018;
  geometry_msgs::Pose pose;
  pose.position.x = 0.4;
  pose.position.y = 1.0;
  pose.position.z = 0.009;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  cylinder_object.id = "blue_totem_low";
  object_in_map_pub_.publish(cylinder_object);
  }

  {
  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "blue_totem_middle";
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "map";
  cylinder_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = .25;
  object.dimensions[1] = .25;
  object.dimensions[2] = .018;
  geometry_msgs::Pose pose;
  pose.position.x = 0.4;
  pose.position.y = 1.0;
  pose.position.z = 0.009 + 0.018 + 0.0545;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  cylinder_object.id = "blue_totem_middle";
  object_in_map_pub_.publish(cylinder_object);
  }

 {
  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "blue_totem_top";
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "map";
  cylinder_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = .25;
  object.dimensions[1] = .25;
  object.dimensions[2] = .018;
  geometry_msgs::Pose pose;
  pose.position.x = 0.4;
  pose.position.y = 1.0;
  pose.position.z = 0.009 + 0.018 + 0.0545 + 0.018 + 0.0545;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  cylinder_object.id = "blue_totem_top";
  object_in_map_pub_.publish(cylinder_object);
  }

  {
  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "red_totem_low";
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "map";
  cylinder_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = .25;
  object.dimensions[1] = .25;
  object.dimensions[2] = .018;
  geometry_msgs::Pose pose;
  pose.position.x = -0.4;
  pose.position.y = 1.0;
  pose.position.z = 0.009;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  cylinder_object.id = "red_totem_low";
  object_in_map_pub_.publish(cylinder_object);
  }

  {
  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "red_totem_middle";
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "map";
  cylinder_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = .25;
  object.dimensions[1] = .25;
  object.dimensions[2] = .018;
  geometry_msgs::Pose pose;
  pose.position.x = -0.4;
  pose.position.y = 1.0;
  pose.position.z = 0.009 + 0.018 + 0.0545;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  cylinder_object.id = "red_totem_middle";
  object_in_map_pub_.publish(cylinder_object);
  }

 {
  //add the cylinder into the collision space
  arm_navigation_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "red_totem_top";
  cylinder_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "map";
  cylinder_object.header.stamp = ros::Time::now();
  arm_navigation_msgs::Shape object;
  object.type = arm_navigation_msgs::Shape::BOX;
  object.dimensions.resize(3);
  object.dimensions[0] = .25;
  object.dimensions[1] = .25;
  object.dimensions[2] = .018;
  geometry_msgs::Pose pose;
  pose.position.x = -0.4;
  pose.position.y = 1.0;
  pose.position.z = 0.009 + 0.018 + 0.0545 + 0.018 + 0.0545;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);

  cylinder_object.id = "red_totem_top";
  object_in_map_pub_.publish(cylinder_object);
  }


  ROS_INFO("Should have published");

  ros::Duration(2.0).sleep();

  ros::shutdown();
}

