#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
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

#include "common_smart_nav/GetRobotPose.h"

/* TO MODIFY */
#include "eurobot2014_petit_ai/GetObjective.h"
#include "eurobot2014_petit_ai/UpdatePriority.h"
//#include "indomptable_vision/ImageResult.h"
/*****END*****/

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


#define POSITION    	0
#define ANGLE       	1
#define DISTANCE    	2
#define OBJECT     	3
#define RELEASE     	4
#define FIND_OBJECT	5



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
        ros::ServiceClient update_objective;

        list<pair<uint32_t, geometry_msgs::Pose2D> > current_list;
        ros::Publisher delet_pub;

        ros::Subscriber pause_sub;
        ros::Subscriber resume_sub;

        ros::Publisher release_pub;
        ros::Subscriber pathimpossible_sub;

        ros::Subscriber start_sub;
        ros::Subscriber stop_sub;
        ros::Subscriber color_sub;

        ros::ServiceClient get_image_result;

	ros::Subscriber deltafeedback_sub_;
	ros::Subscriber alphafeedback_sub_;

    private:
        void pathCallback(const std_msgs::Empty::ConstPtr & empty);
        void pauseCallback(const std_msgs::Empty::ConstPtr & empty);
        void resumeCallback(const std_msgs::Empty::ConstPtr & empty);
        void pathimpossibleCallback(const std_msgs::Empty::ConstPtr & empty);
        void startCallback(const std_msgs::Empty::ConstPtr & empty);
        void stopCallback(const std_msgs::Empty::ConstPtr & empty);
        void colorCallback(const std_msgs::Int32::ConstPtr & my_int);
	void fill_trees(void);

	void deltaCallback(const std_msgs::Int8::ConstPtr & deltafeedback);
	void alphaCallback(const std_msgs::Int8::ConstPtr & alphafeedback);
	int delta_ok;
	int alpha_ok;

        ros::NodeHandle nh;

        nav_msgs::Path my_path;
        geometry_msgs::PoseStamped final_objective;

        uint8_t state; // Working / Pause / X / X // X / X / X / Position

/* TO MODIFY */
        list<pair<uint32_t, geometry_msgs::Pose2D> > mammouth; 
        list<pair<uint32_t, geometry_msgs::Pose2D> > frescoes;
        list<pair<uint32_t, geometry_msgs::Pose2D> > mammouth_opp;
        list<pair<uint32_t, geometry_msgs::Pose2D> > end; 
/*****END*****/

        int color;
	int started;
	int cpt_objects;
};

MainAI::MainAI()
{

    color = 1;//-1;
    started = 0;

/* TO MODIFY */
    goal_pub = nh.advertise < geometry_msgs::PoseStamped > ("/PETIT/goal", 5);
    object_pub = nh.advertise < geometry_msgs::PoseStamped > ("/PETIT/object_pose", 5);
    alpha_pub = nh.advertise < std_msgs::Int32 > ("/PETIT/alpha_ros", 5);
    delta_pub = nh.advertise < std_msgs::Int32 > ("/PETIT/delta_ros", 5);

    path_sub_ = nh.subscribe < std_msgs::Empty > ("/PETIT/path_done", 20, &MainAI::pathCallback, this);


    pause_sub = nh.subscribe < std_msgs::Empty > ("/PETIT/pause_AI", 20, &MainAI::pauseCallback, this);
    resume_sub = nh.subscribe < std_msgs::Empty > ("/PETIT/resume_AI", 20, &MainAI::resumeCallback, this);


    get_pose = nh.serviceClient<common_smart_nav::GetRobotPose>("/PETIT/get_robot_pose");

    get_objective = nh.serviceClient<eurobot2014_petit_ai::GetObjective>("/PETIT/get_objective");

    update_objective = nh.serviceClient<eurobot2014_petit_ai::UpdatePriority>("/PETIT/update_priority");

    delet_pub = nh.advertise < geometry_msgs::PoseStamped > ("/PETIT/delet_objective", 5);

    release_pub = nh.advertise < std_msgs::Empty > ("/PETIT/release_objects", 5);

    pathimpossible_sub = nh.subscribe < std_msgs::Empty > ("/PETIT/goal_unreachable", 20, &MainAI::pathimpossibleCallback, this);

    start_sub = nh.subscribe < std_msgs::Empty > ("/GENERAL/start_match", 20, &MainAI::startCallback, this);
    stop_sub = nh.subscribe < std_msgs::Empty > ("/GENERAL/stop_match", 20, &MainAI::stopCallback, this);
    color_sub = nh.subscribe < std_msgs::Int32 > ("/GENERAL/color", 20, &MainAI::colorCallback, this);

    //get_image_result = nh.serviceClient<indomptable_vision::ImageResult>("/indomptable/image_result");

    deltafeedback_sub_ = nh.subscribe < std_msgs::Int8 > ("/PETIT/delta_feedback", 1, &MainAI::deltaCallback, this);
    alphafeedback_sub_ = nh.subscribe < std_msgs::Int8 > ("/PETIT/alpha_feedback", 1, &MainAI::alphaCallback, this);
/*****END*****/

    delta_ok = 0;
    alpha_ok = 0;

    final_objective.pose.position.x = 0.0;
    final_objective.pose.position.y = 0.0;
    final_objective.pose.position.z = 0.0;

    state = 0;


    cpt_objects = 0;

    //usleep(2000000);
}

void MainAI::fill_trees(void)
{
/* TO MODIFY */
    geometry_msgs::Pose2D tmp;
    tmp.x = color * (0.750);
    tmp.y = 1.55;
    tmp.theta = 1.5707963 + 0.02;
    mammouth.push_back(make_pair(POSITION, tmp));
    tmp.x = 0;
    tmp.y = 0;
    tmp.theta = 0;
    mammouth.push_back(make_pair(RELEASE, tmp));
    tmp.x = color * (0.750);
    tmp.y = 1.55;
    tmp.theta = 1.5707963;
    mammouth.push_back(make_pair(POSITION, tmp));
    tmp.x = 0;
    tmp.y = 0;
    tmp.theta = 0;
    mammouth.push_back(make_pair(RELEASE, tmp));
    tmp.x = color * (0.750);
    tmp.y = 1.55;
    tmp.theta = 1.5707963 - 0.02;
    mammouth.push_back(make_pair(POSITION, tmp));
    tmp.x = 0;
    tmp.y = 0;
    tmp.theta = 0;
    mammouth.push_back(make_pair(RELEASE, tmp));


    tmp.x = color * (0.150);
    tmp.y = 1.800;
    tmp.theta = 0.0;
    frescoes.push_back(make_pair(POSITION, tmp));
    tmp.x = 0.0;
    tmp.y = +1.000;
    tmp.theta = 0.0;
    frescoes.push_back(make_pair(DISTANCE, tmp));
    tmp.x = 0.0;
    tmp.y = -1.000;
    tmp.theta = 0.0;
    frescoes.push_back(make_pair(DISTANCE, tmp));


    tmp.x = -color * (0.750);
    tmp.y = 1.55;
    tmp.theta = 1.5707963 + 0.02;
    mammouth_opp.push_back(make_pair(POSITION, tmp));
    tmp.x = 0;
    tmp.y = 0;
    tmp.theta = 0;
    mammouth_opp.push_back(make_pair(RELEASE, tmp));
    tmp.x = -color * (0.750);
    tmp.y = 1.55;
    tmp.theta = 1.5707963;
    mammouth_opp.push_back(make_pair(POSITION, tmp));
    tmp.x = 0;
    tmp.y = 0;
    tmp.theta = 0;
    mammouth_opp.push_back(make_pair(RELEASE, tmp));
    tmp.x = -color * (0.750);
    tmp.y = 1.55;
    tmp.theta = 1.5707963 - 0.02;
    mammouth_opp.push_back(make_pair(POSITION, tmp));
    tmp.x = 0;
    tmp.y = 0;
    tmp.theta = 0;
    mammouth_opp.push_back(make_pair(RELEASE, tmp));


    tmp.x = color * (0.0);
    tmp.y = 1.7;
    tmp.theta = 1.5707963;
    end.push_back(make_pair(POSITION, tmp));
    


    current_list = mammouth;


/*****END*****/

}

void MainAI::deltaCallback(const std_msgs::Int8::ConstPtr & deltafeedback)
{
    delta_ok = deltafeedback->data;
}

void MainAI::alphaCallback(const std_msgs::Int8::ConstPtr & alphafeedback)
{
    alpha_ok = alphafeedback->data;
}




void MainAI::main_loop(void)
{

    geometry_msgs::PoseStamped tmppose;
    tmppose.header.frame_id = "/petit_map";
    std_msgs::Int32 tmpaction;
    std_msgs::Empty tmprelease;
    common_smart_nav::GetRobotPose tmp_pose;
/* TO MODIFY */
    eurobot2014_petit_ai::GetObjective tmp_obj;
    eurobot2014_petit_ai::UpdatePriority update_prio;
    //indomptable_vision::ImageResult tmp_img_res;
/* TO MODIFY */
    
    tmppose.pose.position.x = 0;
    tmppose.pose.position.y = 0;

    if(started) {

    switch(state) {
        case 0b00000000 : // Next objective

            if (get_objective.call(tmp_obj))
            {
                tmppose.pose.position.x = tmp_obj.response.goal.pose.position.x;
                tmppose.pose.position.y = tmp_obj.response.goal.pose.position.y;
                final_objective = tmp_obj.response.goal;
            }
            else
            {
                ROS_ERROR("Failed to call service GetObjective");
            }


/* TO MODIFY */
            if( (tmppose.pose.position.x == color*(0.750)) && (tmppose.pose.position.y == 1.550) ) {
                current_list = mammouth;
                ROS_ERROR("Go kill your mammouth !");
            }
            if( (tmppose.pose.position.x == -color*(0.750)) && (tmppose.pose.position.y == 1.550) ) {
                current_list = mammouth_opp;
                ROS_ERROR("Go kill opponent mammouth !");
            }
            if( (tmppose.pose.position.x == color*(0.150)) && (tmppose.pose.position.y == 1.800) ) {
                current_list = frescoes;
                ROS_ERROR("Put the frescoes");
            }
            if( (tmppose.pose.position.x == color*(0)) && (tmppose.pose.position.y == 1.700) ) {
                current_list = end;
                ROS_ERROR("final position");
            }
/*****END*****/

            //current_list = totem_self_n;
            state = 0b10000000;
            break;
        case 0b10000000 : // Next action for the objective
            if(!current_list.empty()) {
                switch(current_list.front().first) {
                    case POSITION :
                        tmppose.header.frame_id = "/petit_map";
                        tmppose.pose.position.x = current_list.front().second.x;
                        tmppose.pose.position.y = current_list.front().second.y;
                        rotate(0.0, current_list.front().second.theta, 0.0, &tmppose);
                        goal_pub.publish(tmppose);
                        ROS_ERROR("Sending pose %f %f", tmppose.pose.position.x, tmppose.pose.position.y);
                        state += 1;
			current_list.pop_front();
                        break;
                    case ANGLE :
			usleep(200000);
                        tmpaction.data = (current_list.front().second.theta * 1000);
                        //alpha_pub.publish(tmpaction);

			alpha_ok = 0;
			alpha_pub.publish(tmpaction);
			usleep(50000);
			ros::spinOnce();
			usleep(50000);
			ros::spinOnce();
			while(alpha_ok == 0) {
				alpha_pub.publish(tmpaction);
				usleep(50000);
				ros::spinOnce();
				usleep(50000);
				ros::spinOnce();
			}


                        ROS_ERROR("Sending angle %d", tmpaction.data);
			usleep(1200000);
			current_list.pop_front();
                        break;
                    case DISTANCE :
			usleep(300000);
/*
                        if (current_list.front().second.theta == 0.0) {

                            if (get_pose.call(tmp_pose))
                            {
                                //ROS_INFO("Sum: %ld", get_path.response.plan);
                                tmppose.pose.position.x = tmp_pose.response.pose.pose.position.x;
                                tmppose.pose.position.y = tmp_pose.response.pose.pose.position.y;
                                tmpaction.data = (sqrt( pow(current_list.front().second.x - tmppose.pose.position.x, 2) + pow(current_list.front().second.y - tmppose.pose.position.y, 2)) * 1000) ;
                                
				delta_ok = 0;
				delta_pub.publish(tmpaction);
				usleep(50000);
				ros::spinOnce();
				usleep(50000);
				ros::spinOnce();
				while(delta_ok == 0) {
					delta_pub.publish(tmpaction);
					usleep(50000);
					ros::spinOnce();
					usleep(50000);
					ros::spinOnce();
				}


                                ROS_ERROR("Sending distance %d", tmpaction.data);
                            }
                            else
                            {
                                ROS_ERROR("Failed to call service GetRobotPose");
                            }
                            usleep(700000);
                        }
                        else {
                            tmpaction.data = current_list.front().second.theta * 1000;
                            delta_pub.publish(tmpaction);
                            ROS_ERROR("Sending distance %d", tmpaction.data);
                            usleep(800000);
                        }
*/
			current_list.pop_front();
                        break;
                    case OBJECT :
                        tmppose.pose.position.x = current_list.front().second.x;
                        tmppose.pose.position.y = current_list.front().second.y;
                        tmppose.pose.position.z = current_list.front().second.theta;
                        object_pub.publish(tmppose);
                        ROS_ERROR("Sending object %f %f %f", tmppose.pose.position.x, tmppose.pose.position.y, tmppose.pose.position.z);
                        //usleep(5000000);
                        usleep(50000);

			if(tmppose.pose.position.z == 0.062) {
				usleep(1000000);
			}

                        //state += 2;
                        tmppose.pose.position.x = color*(1.500 - 0.250);
                        tmppose.pose.position.y = 0.800;
                        tmppose.pose.position.z = 0.0;
                        update_prio.request.goal = tmppose;
                        tmpaction.data = 2;
                        update_prio.request.prio = tmpaction;

                        if (update_objective.call(update_prio))
                        {  
                        }   
                        else
                        {
                            ROS_ERROR("Failed to call service UpdatePriority");
                        }

			current_list.pop_front();
                        break;
                    case RELEASE :
                        release_pub.publish(tmprelease);
                        ROS_ERROR("Releasing objects");
			current_list.pop_front();
                        usleep(1000000);
                        break;
                    case FIND_OBJECT :

                        usleep(200000);
/* TO MODIFY */
/*
			if(current_list.front().second.theta == 1) { // Find CD
                                        ROS_ERROR("Trying to find CD");
				
				tmp_img_res.request.type.data = 1;
				if (get_image_result.call(tmp_img_res))
				{   
					if((tmp_img_res.response.type.data == 1) && (cpt_objects != 3)) {
						tmppose.pose.position.x = tmp_img_res.response.x.data;
						tmppose.pose.position.y = tmp_img_res.response.y.data;
						tmppose.pose.position.z = 0.072;
						object_pub.publish(tmppose);
						ROS_ERROR("Sending object %f %f %f", tmppose.pose.position.x, tmppose.pose.position.y, tmppose.pose.position.z);
						usleep(3000000);
						cpt_objects++;
						
					}
					else {
						current_list.pop_front();
						cpt_objects = 0;
                                        ROS_ERROR("NO CD");

					}
				}
				else
				{   
                                        ROS_ERROR("Failed to call service ImageResult");
					current_list.pop_front();
						cpt_objects = 0;
				}

				
			}
			else if(current_list.front().second.theta == 2) { // Find BAR
                                        ROS_ERROR("Trying to find BAR");

                                tmp_img_res.request.type.data = 2;
                                if (get_image_result.call(tmp_img_res))
                                {
                                        if(tmp_img_res.response.type.data == 2) {
                                                tmppose.pose.position.x = tmp_img_res.response.x.data;
                                                tmppose.pose.position.y = tmp_img_res.response.y.data;
                                                tmppose.pose.position.z = 0.062;
                                                object_pub.publish(tmppose);
                                                ROS_ERROR("Sending object %f %f %f", tmppose.pose.position.x, tmppose.pose.position.y, tmppose.pose.position.z);
                                                usleep(3000000);

                                        }
                                        else {
                                                current_list.pop_front();
						ROS_ERROR("No BAR");
						cpt_objects = 0;

                                        }
                                }
                                else
                                {
                                        ROS_ERROR("Failed to call service ImageResult");
					current_list.pop_front();
					cpt_objects = 0;
                                }


			} 
			else {
				current_list.pop_front();
				cpt_objects = 0;
			}
*/
/*****END*****/
                        break;
                    default :
			current_list.pop_front();
                        break;
                }

		if(current_list.size() < 2) { // One more element => delet the objective
			delet_pub.publish(final_objective);
		}


            }
            else {
                //update_prio.goal = final_objective;
                //delet_pub.publish(final_objective);
                //usleep(300000);
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

void MainAI::pathimpossibleCallback(const std_msgs::Empty::ConstPtr & empty)
{
    // Search another goal
    if(state != 0b00000000) {
        state = 0;
	ROS_ERROR("Search another goal.");
    }
}

void MainAI::pathCallback(const std_msgs::Empty::ConstPtr & empty)
{
    usleep(400000);
    if(state & 0b00000001) {
        state -= 1;
    }
}

void MainAI::pauseCallback(const std_msgs::Empty::ConstPtr & empty)
{
    if( state & 0b01000000 ) {
        //state -= 1;
    }
    else {
        state += 0b01000000;
    }
}

void MainAI::resumeCallback(const std_msgs::Empty::ConstPtr & empty)
{
    if( state & 0b01000000 ) {
        state -= 0b01000000;
    }
    else {
        //state += 0b01000000;
    }
}

void MainAI::startCallback(const std_msgs::Empty::ConstPtr & empty)
{
	started = 1;
	//system("gst-launch-0.10 filesrc location=/home/jbot/Documents/robo_theme.mp3 ! mad ! audioconvert ! audioresample ! alsasink &");
}

void MainAI::stopCallback(const std_msgs::Empty::ConstPtr & empty)
{
	started = 0;
}

void MainAI::colorCallback(const std_msgs::Int32::ConstPtr & my_int)
{
	if(my_int->data == 1) {
		color = my_int->data;
	}
	else {
		color = -1;
	}

	fill_trees();
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
/* TO MODIFY */
    ros::init(argc, argv, "PETIT_main_ai");
/*****END*****/
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



