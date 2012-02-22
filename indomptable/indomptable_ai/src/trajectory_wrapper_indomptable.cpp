#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetPlan.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "indomptable_nav/GetRobotPose.h"

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
#include <list>


class TrajectoryManager {
  public:
    TrajectoryManager();
    void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose);

    // Goal suscriber
    ros::Subscriber goal_sub_;

    ros::Publisher path_pub;

    ros::ServiceClient get_pose;
    ros::ServiceClient get_path;

  private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
     ros::NodeHandle nh;

     nav_msgs::Path my_path;
     geometry_msgs::Pose2D final_pose;

};

TrajectoryManager::TrajectoryManager()
{

    // Goal suscriber
    goal_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("/move_base/goal_test", 20, &TrajectoryManager::goalCallback, this);

    get_pose = nh.serviceClient<indomptable_nav::GetRobotPose>("/indomptable/get_robot_pose");
    get_path = nh.serviceClient<nav_msgs::GetPlan>("/move_base_indomptable/NavfnROS/make_plan");

    path_pub = nh.advertise < nav_msgs::Path > ("/my_indomptable_path", 50);

    my_path.poses = std::vector < geometry_msgs::PoseStamped > ();

    if (my_path.poses.std::vector < geometry_msgs::PoseStamped >::size() >
        (my_path.poses.std::vector < geometry_msgs::PoseStamped >::max_size() - 2)) {
        my_path.poses.std::vector < geometry_msgs::PoseStamped >::pop_back();
    }

    final_pose.x = 0.0;
    final_pose.y = 0.14;
    final_pose.theta = 0.0;

}


void TrajectoryManager::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose)
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

void TrajectoryManager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & pose)
{
    final_pose.x = pose->pose.position.x;
    final_pose.y = pose->pose.position.y;

    nav_msgs::GetPlan tmp_plan;

    indomptable_nav::GetRobotPose tmp_pose;

    if (get_pose.call(tmp_pose))
    {
            //ROS_INFO("Sum: %ld", get_path.response.plan);
        tmp_plan.request.start = tmp_pose.response.pose;
    }
    else
    {
            ROS_ERROR("Failed to call service GetRobotPose");
    }


    //tmp_plan.request.start = 90;
    tmp_plan.request.goal = *pose;
    tmp_plan.request.tolerance = 0.01;
    if (get_path.call(tmp_plan))
    {
            //ROS_INFO("Sum: %ld", get_path.response.plan);
	path_pub.publish(tmp_plan.response.plan);
    }
    else
    {
            ROS_ERROR("Failed to call service GetPlan");
    }


    ROS_INFO("Trajectory manager : Goal sent.");

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
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
        ros::init(argc, argv, "indomptable_trajectory_manager");
        TrajectoryManager trajectorymanager;
        tf::TransformListener listener(ros::Duration(10));

        // Refresh rate
        ros::Rate loop_rate(50);
        float rotation = 0.0;
        while (ros::ok()) {

                ros::spinOnce();
                loop_rate.sleep();

        }

        ros::Duration(2.0).sleep();

        ros::shutdown();
}



