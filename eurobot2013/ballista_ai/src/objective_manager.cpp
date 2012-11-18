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

#include "ballista_nav/GetRobotPose.h"
#include "ballista_ai/GetObjective.h"
#include "ballista_ai/UpdatePriority.h"

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
#include <map>

#define ROBOT_NAME ballista

using namespace std;


class ObjectiveManager {
    public:
        ObjectiveManager();
        void rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose);
        void loop(void);

        ros::Publisher goal_debug;
        ros::Publisher celoxia_goal_debug;
        ros::Publisher path_debug;

        ros::ServiceClient get_path;
        ros::ServiceClient get_pose;

        ros::ServiceServer goal_service;

        ros::Subscriber delet_sub;
        ros::ServiceServer update_prio_service;

        ros::Subscriber color_sub;

        ros::ServiceClient celoxia_get_path;
        ros::ServiceClient celoxia_get_pose;
        ros::ServiceServer celoxia_goal_service;
        ros::ServiceServer celoxia_update_prio_service;

    private:
        double compute_distance(nav_msgs::Path path_to_compute);
        bool getObjective(ballista_ai::GetObjective::Request  &req, ballista_ai::GetObjective::Response &res );
        bool updateObjective(ballista_ai::UpdatePriority::Request  &req, ballista_ai::UpdatePriority::Response &res );
        void deletCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
        void fill_trees(void);
        void colorCallback(const std_msgs::Int32::ConstPtr & my_int);

        bool celoxia_getObjective(ballista_ai::GetObjective::Request  &req, ballista_ai::GetObjective::Response &res );
        bool celoxia_updateObjective(ballista_ai::UpdatePriority::Request  &req, ballista_ai::UpdatePriority::Response &res );


        ros::NodeHandle nh;


        geometry_msgs::PoseStamped best_objective;
        geometry_msgs::PoseStamped celoxia_best_objective;

        //map<geometry_msgs::Pose2D, uint32_t> objectives;
        list< pair<geometry_msgs::PoseStamped, uint32_t> > old_objectives;
        list< pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> > > objectives;

        int color;
};

ObjectiveManager::ObjectiveManager()
{
    color = 1;

    goal_debug = nh.advertise < geometry_msgs::PoseStamped > ("/best_objective", 10);
    celoxia_goal_debug = nh.advertise < geometry_msgs::PoseStamped > ("/celoxia_best_objective", 10);
    path_debug = nh.advertise < nav_msgs::Path > ("/debug_path", 10);

    get_pose = nh.serviceClient<ballista_nav::GetRobotPose>("/ballista/get_robot_pose");
    get_path = nh.serviceClient<nav_msgs::GetPlan>("/move_base_ballista/NavfnROS/make_plan");

    goal_service = nh.advertiseService("/ballista/get_objective", &ObjectiveManager::getObjective, this);

    delet_sub = nh.subscribe < geometry_msgs::PoseStamped > ("/delet_objective", 10, &ObjectiveManager::deletCallback, this);

    update_prio_service = nh.advertiseService("/ballista/update_priority", &ObjectiveManager::updateObjective, this);

    color_sub = nh.subscribe < std_msgs::Int32 > ("/color", 20, &ObjectiveManager::colorCallback, this);


    celoxia_get_path = nh.serviceClient<nav_msgs::GetPlan>("/move_base_celoxia/NavfnROS/make_plan");
    celoxia_get_pose = nh.serviceClient<ballista_nav::GetRobotPose>("/celoxia/get_robot_pose");
    celoxia_goal_service = nh.advertiseService("/celoxia/get_objective", &ObjectiveManager::celoxia_getObjective, this);
    celoxia_update_prio_service = nh.advertiseService("/celoxia/update_priority", &ObjectiveManager::celoxia_updateObjective, this);



    best_objective.header.frame_id = "/map";
    best_objective.pose.position.x = 0.0;
    best_objective.pose.position.y = 0.0;
    best_objective.pose.position.z = 0.0;

    celoxia_best_objective.header.frame_id = "/map";
    celoxia_best_objective.pose.position.x = 0.0;
    celoxia_best_objective.pose.position.y = 0.0;
    celoxia_best_objective.pose.position.z = 0.0;






}

void ObjectiveManager::fill_trees(void)
{   

    geometry_msgs::PoseStamped tmp_obj;
    tmp_obj.header.frame_id = "/map";
    tmp_obj.pose.position.z = 0;

    tmp_obj.pose.position.x = color*(1.500 - 0.600); // GIFT 1
    tmp_obj.pose.position.y = 1.000; 
    // Priority : 20
    objectives.push_back( pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> >(tmp_obj, pair<uint32_t, uint32_t>(5, 200) ) );

    tmp_obj.pose.position.x = color*(1.500 - 0.250); // GIFT 2
    tmp_obj.pose.position.y = 0.800;
    // Priority : 1
    objectives.push_back( pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> >(tmp_obj, pair<uint32_t, uint32_t>(5, 100) ) );


    tmp_obj.pose.position.x = color*(1.500 - 0.640); // GIFT 3
    tmp_obj.pose.position.y = 1.700;
    // Priority : 8
    objectives.push_back( pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> >(tmp_obj, pair<uint32_t, uint32_t>(5, 50) ) );


    tmp_obj.pose.position.x = color*(-1.500 + 0.640 + 0.477); // GIFT 4
    tmp_obj.pose.position.y = 1.700;
    // Priority : 8
    objectives.push_back( pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> >(tmp_obj, pair<uint32_t, uint32_t>(5, 40) ) );


    tmp_obj.pose.position.x = color*(1.500 - 0.250); // CAKE 1
    tmp_obj.pose.position.y = 0.800;
    // Priority : 1
    objectives.push_back( pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> >(tmp_obj, pair<uint32_t, uint32_t>(30, 0) ) );

    
    tmp_obj.pose.position.x = color*(0); // CAKE 2
    tmp_obj.pose.position.y = 2.000 - 0.347;
    // Priority : 3
    objectives.push_back( pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> >(tmp_obj, pair<uint32_t, uint32_t>(30, 0) ) );


    tmp_obj.pose.position.x = -color*(1.500 - 0.250); // CAKE 3
    tmp_obj.pose.position.y = 0.800;
    // Priority : 1
    objectives.push_back( pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> >(tmp_obj, pair<uint32_t, uint32_t>(30, 0) ) );

}

void ObjectiveManager::colorCallback(const std_msgs::Int32::ConstPtr & my_int)
{
        if(my_int->data == 1) {
                color = my_int->data;
        }
        else { 
                color = -1;
        }

        fill_trees();
}



bool ObjectiveManager::getObjective(ballista_ai::GetObjective::Request  &req,
        ballista_ai::GetObjective::Response &res )
{
    res.goal = best_objective;
    //res.sum = req.a + req.b;
    //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

bool ObjectiveManager::celoxia_getObjective(ballista_ai::GetObjective::Request  &req,
        ballista_ai::GetObjective::Response &res )
{
    res.goal = celoxia_best_objective;
    //res.sum = req.a + req.b;
    //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    //ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

bool ObjectiveManager::updateObjective(ballista_ai::UpdatePriority::Request  &req, ballista_ai::UpdatePriority::Response &res )
{

    list< pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> > > tmp_list;
    list< pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> > > tmp_list2;

    tmp_list = objectives;

    while (!tmp_list.empty()) {

        if( (tmp_list.back().first.pose.position.x == req.goal.pose.position.x) && (tmp_list.back().first.pose.position.y == req.goal.pose.position.y) ) {

            tmp_list2.push_back( pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> >(tmp_list.back().first, pair<uint32_t, uint32_t>(tmp_list.back().second.first + req.prio.data, tmp_list.back().second.second) ) );
        }
        else {
            tmp_list2.push_back(tmp_list.back());
        }
        tmp_list.pop_back();

    }

    objectives = tmp_list2;

}

bool ObjectiveManager::celoxia_updateObjective(ballista_ai::UpdatePriority::Request  &req, ballista_ai::UpdatePriority::Response &res )
{

    list< pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> > > tmp_list;
    list< pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> > > tmp_list2;

    tmp_list = objectives;

    while (!tmp_list.empty()) {

        if( (tmp_list.back().first.pose.position.x == req.goal.pose.position.x) && (tmp_list.back().first.pose.position.y == req.goal.pose.position.y) ) {

              tmp_list2.push_back( pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> >(tmp_list.back().first, pair<uint32_t, uint32_t>(tmp_list.back().second.first, tmp_list.back().second.second + req.prio.data) ) );
        }
        else {
            tmp_list2.push_back(tmp_list.back());
        }
        tmp_list.pop_back();

    }

    objectives = tmp_list2;

}


void ObjectiveManager::deletCallback(const geometry_msgs::PoseStamped::ConstPtr & pose) 
{

    list< pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> > > tmp_list;
    list< pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> > > tmp_list2;

    tmp_list = objectives;

    while (!tmp_list.empty()) {

        if( (tmp_list.back().first.pose.position.x == pose->pose.position.x) && (tmp_list.back().first.pose.position.y == pose->pose.position.y) ) {
/* Cas special
            if( (tmp_list.back().first.pose.position.x == (color*(1.500 - 0.250)) ) && (tmp_list.back().first.pose.position.y == 0.800) ) {
                tmp_list2.push_back( pair<geometry_msgs::PoseStamped, uint32_t>(tmp_list.back().first, 0) );
            }
*/
        }
        else {
            tmp_list2.push_back(tmp_list.back());
        }
        tmp_list.pop_back();

    }

    objectives = tmp_list2;

    ObjectiveManager::loop();

}


void ObjectiveManager::rotate(double heading, double attitude, double bank, geometry_msgs::PoseStamped * pose)
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

double ObjectiveManager::compute_distance(nav_msgs::Path path_to_compute) 
{
    double distance = 0;
    geometry_msgs::Pose2D current_pose;

    if ( !(path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::empty()) ) {
        current_pose.x = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x;
        current_pose.y = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y;
        path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::erase (path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::begin());

        while ( !(path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::empty()) ) {
            distance += sqrt( pow(current_pose.x - path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x, 2) + pow(current_pose.y - path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y, 2) );

            current_pose.x = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.x;
            current_pose.y = path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::front().pose.position.y;
            path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::erase (path_to_compute.poses.std::vector<geometry_msgs::PoseStamped >::begin());

        }

    }
    //ROS_ERROR("Distance : %f", distance);

    return distance;

}

void ObjectiveManager::loop(void) 
{

    double best_prio = 0;
    double current_prio = 0;
    double celoxia_best_prio = 0;
    double celoxia_current_prio = 0;

    nav_msgs::GetPlan tmp_plan;

    ballista_nav::GetRobotPose tmp_pose;

    //loop
    for( list< pair<geometry_msgs::PoseStamped, pair<uint32_t, uint32_t> > >::iterator iter = objectives.begin(); iter != objectives.end(); iter++ ) {
        current_prio = 0;
        if (get_pose.call(tmp_pose))
        {  
            //ROS_INFO("Sum: %ld", get_path.response.plan);
            tmp_plan.request.start = tmp_pose.response.pose;
        }
        else
        {
            ROS_ERROR("Failed to call service GetRobotPose");
        }
         

        //ROS_ERROR("Prio : %d", iter->second);
        //tmp_plan.request.start = 90;
        tmp_plan.request.goal = iter->first;
        tmp_plan.request.tolerance = 0.01;
        if (get_path.call(tmp_plan))
        {  
            //ROS_INFO("Sum: %ld", get_path.response.plan);
            //path_debug.publish(tmp_plan.response.plan);

            // compute distance to goal 
            current_prio = (3.0) / (ObjectiveManager::compute_distance(tmp_plan.response.plan));
            if(current_prio < 10000000.0) {
                // add it to the base priority
                if((double)iter->second.first > 0.0)
                    current_prio = current_prio + (double)iter->second.first;
                else 
                    current_prio = 0.0;
                //ROS_ERROR("Prio tot : %f", current_prio);
                if(current_prio > best_prio) {
                    best_prio = current_prio;
                    best_objective = iter->first;
                }
            }

        }
        else
        {
            ROS_ERROR("Failed to call service GetPlan");
        }



        celoxia_current_prio = 0;
        if (celoxia_get_pose.call(tmp_pose))
        {
            //ROS_INFO("Sum: %ld", get_path.response.plan);
            tmp_plan.request.start = tmp_pose.response.pose;
        }
        else
        {
            ROS_ERROR("Failed to call service GetCeloxiaPose");
        }


        //ROS_ERROR("Prio : %d", iter->second);
        //tmp_plan.request.start = 90;
        tmp_plan.request.goal = iter->first;
        tmp_plan.request.tolerance = 0.01;
        if (celoxia_get_path.call(tmp_plan))
        {
            //ROS_INFO("Sum: %ld", get_path.response.plan);
            //path_debug.publish(tmp_plan.response.plan);

            // compute distance to goal 
            celoxia_current_prio = (3.0) / (ObjectiveManager::compute_distance(tmp_plan.response.plan));
            if(celoxia_current_prio < 10000000.0) {
                // add it to the base priority
                if((double)iter->second.second > 0.0)
                    celoxia_current_prio = celoxia_current_prio + (double)iter->second.second;
                else 
                    celoxia_current_prio = 0.0;
                //ROS_ERROR("Prio tot : %f", current_prio);
                if(celoxia_current_prio > celoxia_best_prio) {
                    celoxia_best_prio = celoxia_current_prio;
                    celoxia_best_objective = iter->first;
                }
            }

        }
        else
        {
            ROS_ERROR("Failed to call service GetPlan");
        }


        //usleep(200000); // For debug purpose
    }
    // end loop

    // if no objective available, make random movements
    if(best_prio > 10000000.0) {
        best_objective.pose.position.x = 0;
        best_objective.pose.position.y = 0;
    }
    if(celoxia_best_prio > 10000000.0) {
        celoxia_best_objective.pose.position.x = 0;
        celoxia_best_objective.pose.position.y = 0;
    }

    // For debug purpose
    goal_debug.publish(best_objective);
    celoxia_goal_debug.publish(celoxia_best_objective);

    //ROS_INFO("Trajectory manager : Goal sent.");


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
    ros::init(argc, argv, "objective_manager");
    ObjectiveManager objectivemanager;
    tf::TransformListener listener(ros::Duration(10));

    // Refresh rate
    ros::Rate loop_rate(2);
    float rotation = 0.0;
    while (ros::ok()) {

        ros::spinOnce();
        loop_rate.sleep();
        objectivemanager.loop();
    }

    ros::Duration(2.0).sleep();

    ros::shutdown();
}



