#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <maximus_position/AvrPose.h>
#include <maximus_position/AvrVel.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

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


class TransformPose {
  public:
    TransformPose();
    void publish_all(tf::TransformListener& listener);

//     ros::Subscriber tf_sub_;
//     ros::Publisher pose_pub;
     ros::Publisher odom_pub;
     ros::Subscriber xspeed_sub_;
     ros::Subscriber tspeed_sub_;


  private:
//    void tfCallback(const tf::tfMessage::ConstPtr & mytf);
    void xspeedCallback(const std_msgs::Float32::ConstPtr & pose);
    void tspeedCallback(const std_msgs::Float32::ConstPtr & pose);
    void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose);
     ros::NodeHandle nh;

     nav_msgs::Odometry my_maximus_odom;
     double old_x;
     double old_y;
     double old_theta;
     ros::Time old_time;
     std_msgs::Float32 xspeed;
     std_msgs::Float32 tspeed;

};

TransformPose::TransformPose()
{
/*
    // Joystick suscriber
    avrpose_sub_ = nh.subscribe < maximus_position::AvrPose > ("avrpose", 30, &TransformPose::poseCallback, this);
    // Set a Laser scan sensor for the robot
    pose_pub = nh.advertise < geometry_msgs::PoseStamped > ("avr_maximus_pose", 40);

    cmd_vel_sub_ = nh.subscribe < geometry_msgs::Twist > ("cmd_vel", 30, &TransformPose::velCallback, this);
*/
    odom_pub = nh.advertise < nav_msgs::Odometry > ("odom", 50);
    xspeed_sub_ = nh.subscribe < std_msgs::Float32 > ("/xspeed", 30, &TransformPose::xspeedCallback, this);
    tspeed_sub_ = nh.subscribe < std_msgs::Float32 > ("/tspeed", 30, &TransformPose::tspeedCallback, this);

//    avrvel_pub = nh.advertise < maximus_position::AvrVel > ("avrvel", 30);
/*
    my_maximus_pose.header.frame_id = "/odom";
    my_maximus_pose.header.stamp = ros::Time::now();

    my_maximus_pose.pose.position.x = 0;
    my_maximus_pose.pose.position.y = 0;
    my_maximus_pose.pose.position.z = 0;
    my_maximus_pose.pose.orientation.x = 0;
    my_maximus_pose.pose.orientation.y = 0;
    my_maximus_pose.pose.orientation.z = 0;
    my_maximus_pose.pose.orientation.w = 0;
*/
    my_maximus_odom.header.frame_id = "/odom";
    my_maximus_odom.child_frame_id = "/base_link";
    my_maximus_odom.header.stamp = ros::Time::now();

    my_maximus_odom.pose.pose.position.x = 0;
    my_maximus_odom.pose.pose.position.y = 0;
    my_maximus_odom.pose.pose.position.z = 0;
    my_maximus_odom.pose.pose.orientation.x = 0;
    my_maximus_odom.pose.pose.orientation.y = 0;
    my_maximus_odom.pose.pose.orientation.z = 0;
    my_maximus_odom.pose.pose.orientation.w = 0;

    my_maximus_odom.twist.twist.linear.x = 0;
    my_maximus_odom.twist.twist.linear.y = 0;
    my_maximus_odom.twist.twist.angular.z = 0;

    old_x = 0;
    old_y = 0;
    old_theta = 0;
    old_time = my_maximus_odom.header.stamp;

    xspeed.data = 0.0;
    tspeed.data = 0.0;

/*
    my_maximus_vel.vx = 0;
    my_maximus_vel.vy = 0;
    my_maximus_vel.vth = 0;
*/

}

void TransformPose::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose)
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

void TransformPose::xspeedCallback(const std_msgs::Float32::ConstPtr & pose)
{
	xspeed.data = pose->data;
}

void TransformPose::tspeedCallback(const std_msgs::Float32::ConstPtr & pose)
{
	tspeed.data = pose->data;
}
/*
void TransformPose::poseCallback(const maximus_position::AvrPose::ConstPtr & pose)
{
    float tmp = 0.0;

    my_maximus_pose.header.stamp = ros::Time::now();

    my_maximus_pose.pose.position.x = pose->x;
    my_maximus_pose.pose.position.y = pose->y;
    my_maximus_pose.pose.position.z = 0;

    //TransformPose::rotate(0, (pose->theta), 0, &my_maximus_pose);
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(pose->theta);
    my_maximus_pose.pose.orientation = quat;

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = my_maximus_pose.header.stamp;
    odom_trans.header.frame_id = "/odom";
    odom_trans.child_frame_id = "/base_link";

    odom_trans.transform.translation.x = pose->x;
    odom_trans.transform.translation.y = pose->y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);


    my_maximus_odom.header.stamp = my_maximus_pose.header.stamp;
    my_maximus_odom.pose.pose.position.x = pose->x;
    my_maximus_odom.pose.pose.position.y = pose->y;
    my_maximus_odom.pose.pose.position.z = 0;
    my_maximus_odom.pose.pose.orientation = quat;

    my_maximus_odom.twist.twist.linear.x = pose->vx;
    my_maximus_odom.twist.twist.linear.y = pose->vy;
    my_maximus_odom.twist.twist.linear.z = 0;
    my_maximus_odom.twist.twist.angular.x = 0;
    my_maximus_odom.twist.twist.angular.y = 0;
    my_maximus_odom.twist.twist.angular.z = pose->vth;


    pose_pub.publish(my_maximus_pose);
    odom_pub.publish(my_maximus_odom);

}

void TransformPose::velCallback(const geometry_msgs::Twist::ConstPtr & vel)
{


    my_maximus_vel.vx = ((float) ((int)(vel->linear.x * 20) )) / (float)20;
    my_maximus_vel.vy = ((float) ((int)(vel->linear.y * 20) )) / (float)20;
    my_maximus_vel.vth = ((float) ((int)(vel->angular.z * 20) )) / (float)20;

    avrvel_pub.publish(my_maximus_vel);

}
*/
void TransformPose::publish_all(tf::TransformListener& listener)
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
                listener.waitForTransform("/odom", "/base_link", now, ros::Duration(5.0));
                geometry_msgs::PoseStamped base_pose;
                listener.transformPose("/odom", odom_pose, base_pose);

		my_maximus_odom.header.stamp = base_pose.header.stamp;
		my_maximus_odom.pose.pose.position.x = base_pose.pose.position.x;
		my_maximus_odom.pose.pose.position.y = base_pose.pose.position.y;
		my_maximus_odom.pose.pose.position.z = base_pose.pose.position.z;
		my_maximus_odom.pose.pose.orientation = base_pose.pose.orientation;

		//my_maximus_odom.twist.twist.linear.x = sqrt(pow(base_pose.pose.position.x - old_x, 2) + pow(base_pose.pose.position.y - old_y, 2)) / (my_maximus_odom.header.stamp.toSec() - old_time.toSec());
		my_maximus_odom.twist.twist.linear.x = xspeed.data;
		my_maximus_odom.twist.twist.linear.y = 0;
		my_maximus_odom.twist.twist.linear.z = 0;
		my_maximus_odom.twist.twist.angular.x = 0;
		my_maximus_odom.twist.twist.angular.y = 0;
		my_maximus_odom.twist.twist.angular.z = tspeed.data;
		//my_maximus_odom.twist.twist.angular.z = 1.1 * (tf::getYaw (my_maximus_odom.pose.pose.orientation) - old_theta) / (my_maximus_odom.header.stamp.toSec() - old_time.toSec());

		odom_pub.publish(my_maximus_odom);

		old_x = base_pose.pose.position.x;
		old_y = base_pose.pose.position.y;
		old_theta = tf::getYaw (my_maximus_odom.pose.pose.orientation);
		old_time = my_maximus_odom.header.stamp;

                ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                        my_maximus_odom.pose.pose.position.x, my_maximus_odom.pose.pose.position.y, my_maximus_odom.pose.pose.position.z,
                        my_maximus_odom.twist.twist.linear.x, my_maximus_odom.twist.twist.linear.y, my_maximus_odom.twist.twist.angular.z, base_pose.header.stamp.toSec());

        }
        catch(tf::TransformException& ex){
                ROS_ERROR("Received an exception trying to transform a point from \"odom\" to \"base_link\": %s", ex.what());
        }


}


int main(int argc, char **argv)
{


    ros::init(argc, argv, "transformPose");
    TransformPose transform_Pose;
    // Refresh rate
    ros::Rate loop_rate(30);                                // 35 with bluetooth

    tf::TransformListener listener(ros::Duration(10));


    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        transform_Pose.publish_all(boost::ref(listener));

    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}

