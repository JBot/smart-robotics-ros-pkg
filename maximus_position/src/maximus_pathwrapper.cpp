#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <joy/Joy.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

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

#define BAUDRATE B115200

class MaximusPath {
  public:
    MaximusPath();
    void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose);
    void compute_next_pathpoint(void);

    // Goal suscriber
    ros::Subscriber goal_sub_;
    // Path suscriber
    ros::Subscriber path_sub_;

    ros::Publisher pose2D_pub;
    ros::Publisher poseArray_pub;

  private:
    void pathCallback(const nav_msgs::Path::ConstPtr & path);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
     ros::NodeHandle nh;

     nav_msgs::Path my_maximus_path;
     geometry_msgs::Pose2D final_pose;

    char cpt_send;
};

MaximusPath::MaximusPath()
{

    // Goal suscriber
    goal_sub_ = nh.subscribe < geometry_msgs::PoseStamped > ("/move_base_simple/goal", 20, &MaximusPath::goalCallback, this);
    // Path suscriber
    //path_sub_ = nh.subscribe < nav_msgs::Path > ("/move_base/TrajectoryPlannerROS/global_plan", 20, &MaximusPath::pathCallback, this);
    path_sub_ = nh.subscribe < nav_msgs::Path > ("/move_base/NavfnROS/plan", 20, &MaximusPath::pathCallback, this);

    pose2D_pub = nh.advertise < geometry_msgs::Pose2D > ("/maximus_goal", 50);
    poseArray_pub = nh.advertise < geometry_msgs::PoseArray > ("/poses", 50);

    my_maximus_path.poses = std::vector < geometry_msgs::PoseStamped > ();

    if (my_maximus_path.poses.std::vector < geometry_msgs::PoseStamped >::size() >
        (my_maximus_path.poses.std::vector < geometry_msgs::PoseStamped >::max_size() - 2)) {
        my_maximus_path.poses.std::vector < geometry_msgs::PoseStamped >::pop_back();
    }

    final_pose.x = 0.0;
    final_pose.y = 0.0;
    final_pose.theta = 0.0;

    cpt_send = 0;
}


void MaximusPath::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose)
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

void MaximusPath::pathCallback(const nav_msgs::Path::ConstPtr & path)
{

if(cpt_send == 0) {
	cpt_send++;
}
else {

    ROS_INFO("Path begin.");
    my_maximus_path = *path;
    ROS_INFO("Path next.");

    geometry_msgs::PoseArray my_pose_array;
    int i = 0;

/*    ROS_INFO("%f %f", my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::back().pose.position.x, my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::back().pose.position.y);
    geometry_msgs::Pose tmp_pose;
    tmp_pose.position.x = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::back().pose.position.x;  
    tmp_pose.position.y = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::back().pose.position.y;  
    my_pose_array.poses.std::vector<geometry_msgs::Pose>::push_back(tmp_pose);

    my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::pop_back();

   while( !(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::empty()) ){
        if(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::size() > 5) {
                my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::pop_back();
                my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::pop_back();
                my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::pop_back();
                my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::pop_back();
                ROS_INFO("%f %f", my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::back().pose.position.x, my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::back().pose.position.y);
                geometry_msgs::Pose tmp_pose;
                tmp_pose.position.x = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::back().pose.position.x;  
                tmp_pose.position.y = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::back().pose.position.y;  
                my_pose_array.poses.std::vector<geometry_msgs::Pose>::push_back(tmp_pose);

                my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::pop_back();
        }
        else {
	
                while(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::size() > 1) {
			my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::pop_back();
                }
                ROS_INFO("Not used %f %f", my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::back().pose.position.y);
                my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::pop_back();
        
	}
    //ros::Duration(1.0).sleep();
   } 
*/

/*
		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());


   while( !(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::empty()) ){
	if(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::size() > 5) {
		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
		ROS_INFO("%f %f", my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y);
		geometry_msgs::Pose tmp_pose;
		tmp_pose.position.x = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x;	
		tmp_pose.position.y = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y;	
		my_pose_array.poses.std::vector<geometry_msgs::Pose>::push_back(tmp_pose);

		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
	}
	else {
		while(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::size() > 1) {
			my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
		}
		ROS_INFO("%f %f", my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y);
		geometry_msgs::Pose tmp_pose;
		tmp_pose.position.x = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x;	
		tmp_pose.position.y = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y;	
		my_pose_array.poses.std::vector<geometry_msgs::Pose>::push_back(tmp_pose);
		
		final_pose.x = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x;
    		final_pose.y = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y;
    		MaximusPath::pose2D_pub.publish(final_pose);


		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
	}
    //ros::Duration(1.0).sleep();
   } */
/*
   while(my_pose_array.poses.std::vector<geometry_msgs::Pose>::size() > 9) {
	my_pose_array.poses.std::vector<geometry_msgs::Pose>::pop_back();
   }
*/

//   MaximusPath::poseArray_pub.publish(my_pose_array);
//    ros::Duration(0.5).sleep();
//    ROS_INFO("Path sent.");

   cpt_send = 0;
}

}

void MaximusPath::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & pose)
{
/*    final_pose.x = pose->pose.position.x;
    final_pose.y = pose->pose.position.y;
    MaximusPath::pose2D_pub.publish(final_pose);
    ROS_INFO("Goal sent.");
*/
}

void MaximusPath::compute_next_pathpoint(void) {

   if( !(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::empty()) ){
        if(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::size() > 5) {
                my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
                my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
                my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
                my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
                ROS_INFO("%f %f", my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y);
                
		final_pose.x = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x;
		final_pose.y = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y;
    		MaximusPath::pose2D_pub.publish(final_pose);

		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
        }
        else {
                while(my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::size() > 1) {
                        my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
                }
                ROS_INFO("%f %f", my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y);
                
                final_pose.x = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x;
                final_pose.y = my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y;
                MaximusPath::pose2D_pub.publish(final_pose);

		
		my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::erase (my_maximus_path.poses.std::vector<geometry_msgs::PoseStamped >::begin());
        }
    ros::Duration(0.3).sleep();
    ROS_INFO("Path sent.");
   }

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
    ros::init(argc, argv, "maximus_pathwrapper");
    MaximusPath maximus_path;
    // Refresh rate
    ros::Rate loop_rate(50); 
    float rotation = 0.0;
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
	maximus_path.compute_next_pathpoint();

    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}
