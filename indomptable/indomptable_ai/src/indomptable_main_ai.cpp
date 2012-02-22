#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"
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
#include "indomptable_ai/GetObjective.h"

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
#include <utility>


#define POSITION    0
#define ANGLE       1
#define DISTANCE    2
#define OBJECT      3
#define RELEASE     4



using namespace std;

class MainAI {
    public:
        MainAI();
        void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose);
        void main_loop(void);

        // Goal suscriber
        ros::Subscriber path_sub_;

        ros::Publisher goal_pub;
        ros::Publisher object_pub;
        ros::Publisher alpha_pub;
        ros::Publisher delta_pub;

        ros::ServiceClient get_pose;
        ros::ServiceClient get_path;
        ros::ServiceClient get_objective;

        list<pair<uint32_t, geometry_msgs::Pose2D> > current_list;

    private:
        void pathCallback(const std_msgs::Empty::ConstPtr & empty);
        ros::NodeHandle nh;

        nav_msgs::Path my_path;
        geometry_msgs::Pose2D final_pose;

        uint8_t state;

        list<pair<uint32_t, geometry_msgs::Pose2D> > totem_self_n;
        list<pair<uint32_t, geometry_msgs::Pose2D> > totem_self_s;
        list<pair<uint32_t, geometry_msgs::Pose2D> > totem_opp_n;
        list<pair<uint32_t, geometry_msgs::Pose2D> > totem_opp_s;
        list<pair<uint32_t, geometry_msgs::Pose2D> > bottle_1;
        list<pair<uint32_t, geometry_msgs::Pose2D> > bottle_2;
        list<pair<uint32_t, geometry_msgs::Pose2D> > release;
        list<pair<uint32_t, geometry_msgs::Pose2D> > steal_opp;
        list<pair<uint32_t, geometry_msgs::Pose2D> > gold_middle;
        list<pair<uint32_t, geometry_msgs::Pose2D> > random_move;

        int color;
};

MainAI::MainAI()
{

    color = 1;//-1;

    goal_pub = nh.advertise < geometry_msgs::PoseStamped > ("/move_base/goal_test", 5);
    object_pub = nh.advertise < geometry_msgs::PoseStamped > ("/object_pose", 5);
    alpha_pub = nh.advertise < std_msgs::Int32 > ("/alpha_ros", 5);
    delta_pub = nh.advertise < std_msgs::Int32 > ("/delta_ros", 5);

    path_sub_ = nh.subscribe < std_msgs::Empty > ("/path_done", 20, &MainAI::pathCallback, this);




    get_pose = nh.serviceClient<indomptable_nav::GetRobotPose>("/indomptable/get_robot_pose");

    get_objective = nh.serviceClient<indomptable_ai::GetObjective>("/indomptable/get_objective");

    final_pose.x = 0.0;
    final_pose.y = 0.0;
    final_pose.theta = 0.0;

    state = 0;

    geometry_msgs::Pose2D tmp;
    tmp.x = color * (1.5 - 0.970);
    tmp.y = 0.470;
    tmp.theta = 0;
    totem_self_n.push_back(make_pair(POSITION, tmp));
    tmp.x = color * (1.5 - 1.100);
    tmp.y = 0.500;
    tmp.theta = 0;
    totem_self_n.push_back(make_pair(POSITION, tmp));
    tmp.x = 0.0;
    tmp.y = 0.0;
    tmp.theta = 1.57079;
    totem_self_n.push_back(make_pair(ANGLE, tmp));
    tmp.x = color * (1.5 - 1.100);
    tmp.y = 0.600;
    tmp.theta = 0.0;
    totem_self_n.push_back(make_pair(DISTANCE, tmp));
    tmp.x = 0.12;
    tmp.y = 0.06;
    tmp.theta = 0.072;
    totem_self_n.push_back(make_pair(OBJECT, tmp));

    current_list = totem_self_n;

    usleep(2000000);
}

void MainAI::main_loop(void)
{

    geometry_msgs::PoseStamped tmppose;
    std_msgs::Int32 tmpaction;
    indomptable_nav::GetRobotPose tmp_pose;
    indomptable_ai::GetObjective tmp_obj;
    
    tmppose.pose.position.x = 0;
    tmppose.pose.position.y = 0;

    switch(state) {
        case 0b00000000 : // Next objective

            if (get_objective.call(tmp_obj))
            {
                tmppose.pose.position.x = tmp_obj.response.goal.pose.position.x;
                tmppose.pose.position.y = tmp_obj.response.goal.pose.position.y;
            }
            else
            {
                ROS_ERROR("Failed to call service GetObjective");
            }

            if( (tmppose.pose.position.x == 0.0) && (tmppose.pose.position.y == 0.0) )
                current_list = random_move;
            if( (tmppose.pose.position.x == color*(1.500 - 1.100)) && (tmppose.pose.position.y == 0.600) )
                current_list = totem_self_n;
            if( (tmppose.pose.position.x == color*(1.500 - 1.100)) && (tmppose.pose.position.y == 1.400) )
                current_list = totem_self_s;
            if( (tmppose.pose.position.x == color*(1.500 - 0.250)) && (tmppose.pose.position.y == 0.800) )
                current_list = release;
            if( (tmppose.pose.position.x == color*(1.500 - 0.640)) && (tmppose.pose.position.y == 1.700) )
                current_list = bottle_1;
            if( (tmppose.pose.position.x == color*(-1.500 + 0.640 + 0.477)) && (tmppose.pose.position.y == 1.700) )
                current_list = bottle_2;
            if( (tmppose.pose.position.x == color*(0)) && (tmppose.pose.position.y == (2.000 - 0.647)) )
                current_list = gold_middle;
            if( (tmppose.pose.position.x == -color*(1.500 - 0.250)) && (tmppose.pose.position.y == 0.800) )
                current_list = steal_opp;
            if( (tmppose.pose.position.x == -color*(1.500 - 1.100)) && (tmppose.pose.position.y == 0.600) )
                current_list = totem_opp_n;
            if( (tmppose.pose.position.x == -color*(1.500 - 1.100)) && (tmppose.pose.position.y == 1.400) )
                current_list = totem_opp_s;

            current_list = totem_self_n;
            state = 0b10000000;
            break;
        case 0b10000000 : // Next action for the objective
            if(!current_list.empty()) {
                switch(current_list.front().first) {
                    case POSITION :
                        tmppose.header.frame_id = "/map";
                        tmppose.pose.position.x = current_list.front().second.x;
                        tmppose.pose.position.y = current_list.front().second.y;
                        goal_pub.publish(tmppose);
                        ROS_ERROR("Sending pose %f %f", tmppose.pose.position.x, tmppose.pose.position.y);
                        state += 1;
                        break;
                    case ANGLE :
                        tmpaction.data = (current_list.front().second.theta * 1000);
                        alpha_pub.publish(tmpaction);
                        ROS_ERROR("Sending angle %d", tmpaction.data);
                        usleep(3000000);
                        break;
                    case DISTANCE :

                        if (get_pose.call(tmp_pose))
                        {
                            //ROS_INFO("Sum: %ld", get_path.response.plan);
                            tmppose.pose.position.x = tmp_pose.response.pose.pose.position.x;
                            tmppose.pose.position.y = tmp_pose.response.pose.pose.position.y;
                            tmpaction.data = (sqrt( pow(current_list.front().second.x - tmppose.pose.position.x, 2) + pow(current_list.front().second.y - tmppose.pose.position.y, 2)) * 1000) ;
                            delta_pub.publish(tmpaction);
                            ROS_ERROR("Sending distance %d", tmpaction.data);
                        }
                        else
                        {
                            ROS_ERROR("Failed to call service GetRobotPose");
                        }
                        usleep(3000000);
                        break;
                    case OBJECT :
                        tmppose.pose.position.x = current_list.front().second.x;
                        tmppose.pose.position.y = current_list.front().second.y;
                        tmppose.pose.position.z = current_list.front().second.theta;
                        object_pub.publish(tmppose);
                        ROS_ERROR("Sending object %f %f %f", tmppose.pose.position.x, tmppose.pose.position.y, tmppose.pose.position.z);
                        usleep(5000000);
                        //state += 2;
                        break;
                    default :
                        break;
                }
                current_list.pop_front();
            }
            else {
                state = 0;
            }
            break;
        case 0b10000001 : // Moving
            /*
               if(current_list.front().first == OBJECT) {
               tmppose.pose.position.x = current_list.front().second.x;
               tmppose.pose.position.y = current_list.front().second.y;
               tmppose.pose.position.z = current_list.front().second.theta;
               object_pub.publish(tmppose);
               state += 2;
               }*/
            break;
        default : 
            break;
    }


}

void MainAI::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose)
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

void MainAI::pathCallback(const std_msgs::Empty::ConstPtr & empty)
{
    if(state & 0b00000001) {
        state -= 1;
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
    ros::init(argc, argv, "indomptable_main_ai");
    MainAI mainai;
    tf::TransformListener listener(ros::Duration(10));

    // Refresh rate
    ros::Rate loop_rate(50);
    float rotation = 0.0;
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
        mainai.main_loop();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}



