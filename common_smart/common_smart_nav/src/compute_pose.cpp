#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>

#include "common_smart_nav/GetRobotPose.h"

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


class ComputePose {
  public:
    ComputePose();
    void publish_all(tf::TransformListener& listener);

    bool getRobotPose(common_smart_nav::GetRobotPose::Request  &req, common_smart_nav::GetRobotPose::Response &res );


     ros::Publisher odom_pub;

     ros::ServiceServer pose_service;

  private:
    void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose);
     ros::NodeHandle nh;

     nav_msgs::Odometry my_odom;
     geometry_msgs::PoseStamped robot_pose;
     double old_x;
     double old_y;
     double old_theta;
     ros::Time old_time;

};

ComputePose::ComputePose()
{
    odom_pub = nh.advertise < nav_msgs::Odometry > ("odom", 50);

    my_odom.header.frame_id = "/map";
    my_odom.child_frame_id = "/base_link";
    my_odom.header.stamp = ros::Time::now();

    my_odom.pose.pose.position.x = 0;
    my_odom.pose.pose.position.y = 0;
    my_odom.pose.pose.position.z = 0;
    my_odom.pose.pose.orientation.x = 0;
    my_odom.pose.pose.orientation.y = 0;
    my_odom.pose.pose.orientation.z = 0;
    my_odom.pose.pose.orientation.w = 0;

    my_odom.twist.twist.linear.x = 0;
    my_odom.twist.twist.linear.y = 0;
    my_odom.twist.twist.angular.z = 0;

    old_x = 0;
    old_y = 0;
    old_theta = 0;
    old_time = my_odom.header.stamp;


    //pose_service = nh.advertiseService("get_robot_pose", getRobotPose);
    pose_service = nh.advertiseService("/ROBOT/get_robot_pose", &ComputePose::getRobotPose, this);
}

bool ComputePose::getRobotPose(common_smart_nav::GetRobotPose::Request  &req,
					common_smart_nav::GetRobotPose::Response &res )
{
  res.pose = robot_pose;
  //res.sum = req.a + req.b;
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}



void ComputePose::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose)
{
    // Assuming the angles are in radians.
    double c1 = cos(heading / 2);
    double s1 = sin(heading / 2);
    double c2 = cos(attitude / 2);

    double s2 = sin(attitude / 2);
    double c3 = cos(bank / 2);
    double s3 = sin(bank / 2);
    double c1c2 = c1 * c2;
    double s1s2 = s1 * s2;

    pose->pose.orientation.w = c1c2 * c3 - s1s2 * s3;
    pose->pose.orientation.x = c1c2 * s3 + s1s2 * c3;
    pose->pose.orientation.y = s1 * c2 * c3 + c1 * s2 * s3;
    pose->pose.orientation.z = c1 * s2 * c3 - s1 * c2 * s3;
}

void ComputePose::publish_all(tf::TransformListener& listener)
{

        geometry_msgs::PoseStamped odom_pose;
        odom_pose.header.frame_id = "/base_link";

        //we'll just use the most recent transform available for our simple example
        odom_pose.header.stamp = ros::Time();

        //just an arbitrary point in space
        odom_pose.pose.position.x = 0.0;
        odom_pose.pose.position.y = 0.0;
        odom_pose.pose.position.z = 0.0;

        odom_pose.pose.orientation.x = 0.0;
        odom_pose.pose.orientation.y = 0.0;
        odom_pose.pose.orientation.z = 0.0;
        odom_pose.pose.orientation.w = 1.0;


        try{
                ros::Time now = ros::Time::now();
                listener.waitForTransform("/map", "/base_link", now, ros::Duration(0.5));
                geometry_msgs::PoseStamped base_pose;
                listener.transformPose("/map", odom_pose, base_pose);
		
		robot_pose = base_pose;		

		my_odom.header.stamp = base_pose.header.stamp;
		my_odom.pose.pose.position.x = base_pose.pose.position.x;
		my_odom.pose.pose.position.y = base_pose.pose.position.y;
		my_odom.pose.pose.position.z = base_pose.pose.position.z;
		my_odom.pose.pose.orientation = base_pose.pose.orientation;

		my_odom.twist.twist.linear.x = sqrt(pow(base_pose.pose.position.x - old_x, 2) + pow(base_pose.pose.position.y - old_y, 2)) / (my_odom.header.stamp.toSec() - old_time.toSec());
		//my_odom.twist.twist.linear.x = xspeed.data;
		my_odom.twist.twist.linear.y = 0;
		my_odom.twist.twist.linear.z = 0;
		my_odom.twist.twist.angular.x = 0;
		my_odom.twist.twist.angular.y = 0;
		//my_odom.twist.twist.angular.z = tspeed.data;
		my_odom.twist.twist.angular.z = (tf::getYaw (my_odom.pose.pose.orientation) - old_theta) / (my_odom.header.stamp.toSec() - old_time.toSec());

		odom_pub.publish(my_odom);



		old_x = base_pose.pose.position.x;
		old_y = base_pose.pose.position.y;
		old_theta = tf::getYaw (my_odom.pose.pose.orientation);
		old_time = my_odom.header.stamp;
/*
                ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                        my_maximus_odom.pose.pose.position.x, my_maximus_odom.pose.pose.position.y, my_maximus_odom.pose.pose.position.z,
                        my_maximus_odom.twist.twist.linear.x, my_maximus_odom.twist.twist.linear.y, my_maximus_odom.twist.twist.angular.z, base_pose.header.stamp.toSec());
*/
        }
        catch(tf::TransformException& ex){
                ROS_ERROR("Received an exception trying to transform a point from \"odom\" to \"base_link\": %s", ex.what());
        }


}


int main(int argc, char **argv)
{


    ros::init(argc, argv, "Compute_Pose");
    ComputePose transform_Pose;
    // Refresh rate
    ros::Rate loop_rate(60);                                // 35 with bluetooth

    tf::TransformListener listener(ros::Duration(10));


    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        transform_Pose.publish_all(boost::ref(listener));

    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}

