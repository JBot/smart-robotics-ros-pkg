#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
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

#include "indomptable_nav/ArduGoal.h"

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

using namespace std;


class BallistaSerial {
    public:
        BallistaSerial();

        ros::Subscriber cmd_vel;
        ros::Publisher cmd_vel_out;
        ros::Subscriber cmd_vel_in;

        ros::Subscriber enable_motors;
        ros::Publisher enable_motors_out;
        ros::Subscriber enable_motors_in;

        ros::Subscriber color;
        ros::Publisher color_out;
        ros::Subscriber color_in;

        ros::Subscriber delta;
        ros::Publisher delta_out;
        ros::Subscriber delta_in;

        ros::Subscriber alpha;
        ros::Publisher alpha_out;
        ros::Subscriber alpha_in;

        ros::Subscriber ardugoal;
        ros::Publisher ardugoal_out;
        ros::Subscriber ardugoal_in;

        ros::Subscriber rightfinger;
        ros::Publisher rightfinger_out;
        ros::Subscriber rightfinger_in;

        ros::Subscriber leftfinger;
        ros::Publisher leftfinger_out;
        ros::Subscriber leftfinger_in;

        ros::Subscriber rightuppump;
        ros::Publisher rightuppump_out;
        ros::Subscriber rightuppump_in;

        ros::Subscriber rightdownpump;
        ros::Publisher rightdownpump_out;
        ros::Subscriber rightdownpump_in;

        ros::Subscriber leftuppump;
        ros::Publisher leftuppump_out;
        ros::Subscriber leftuppump_in;

        ros::Subscriber leftdownpump;
        ros::Publisher leftdownpump_out;
        ros::Subscriber leftdownpump_in;





/*
        ros::Publisher goal_debug;
        ros::Publisher celoxia_goal_debug;
        ros::Publisher path_debug;

        ros::Subscriber delet_sub;

        ros::Subscriber color_sub;
*/
    private:
/*        double compute_distance(nav_msgs::Path path_to_compute);
        void deletCallback(const geometry_msgs::PoseStamped::ConstPtr & pose);
        void fill_trees(void);
        void colorCallback(const std_msgs::Int32::ConstPtr & my_int);
*/
        void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr & my_var);
        void cmd_vel_in_cb(const geometry_msgs::Twist::ConstPtr & my_var);
        
        void enable_motors_cb(const std_msgs::Int32::ConstPtr & my_var);
        void enable_motors_in_cb(const std_msgs::Int32::ConstPtr & my_var);

        void color_cb(const std_msgs::Int32::ConstPtr & my_var);
        void color_in_cb(const std_msgs::Int32::ConstPtr & my_var);

        void alpha_cb(const std_msgs::Int32::ConstPtr & my_var);
        void alpha_in_cb(const std_msgs::Int32::ConstPtr & my_var);

        void delta_cb(const std_msgs::Int32::ConstPtr & my_var);
        void delta_in_cb(const std_msgs::Int32::ConstPtr & my_var);

        void ardugoal_cb(const indomptable_nav::ArduGoal::ConstPtr & my_var);
        void ardugoal_in_cb(const indomptable_nav::ArduGoal::ConstPtr & my_var);

        void rightfinger_cb(const std_msgs::Int32::ConstPtr & my_var);
        void rightfinger_in_cb(const std_msgs::Int32::ConstPtr & my_var);

        void leftfinger_cb(const std_msgs::Int32::ConstPtr & my_var);
        void leftfinger_in_cb(const std_msgs::Int32::ConstPtr & my_var);

        void rightuppump_cb(const std_msgs::Int32::ConstPtr & my_var);
        void rightuppump_in_cb(const std_msgs::Int32::ConstPtr & my_var);

        void rightdownpump_cb(const std_msgs::Int32::ConstPtr & my_var);
        void rightdownpump_in_cb(const std_msgs::Int32::ConstPtr & my_var);

        void leftuppump_cb(const std_msgs::Int32::ConstPtr & my_var);
        void leftuppump_in_cb(const std_msgs::Int32::ConstPtr & my_var);

        void leftdownpump_cb(const std_msgs::Int32::ConstPtr & my_var);
        void leftdownpump_in_cb(const std_msgs::Int32::ConstPtr & my_var);





        ros::NodeHandle nh;

    
        geometry_msgs::Twist cmd_vel_value;
        int cmd_vel_flag;

        std_msgs::Int32 enable_motors_value;
        int enable_motors_flag;

        std_msgs::Int32 color_value;
        int color_flag;

        std_msgs::Int32 alpha_value;
        int alpha_flag;

        std_msgs::Int32 delta_value;
        int delta_flag;

        indomptable_nav::ArduGoal ardugoal_value;
        int ardugoal_flag;

        std_msgs::Int32 rightfinger_value;
        int rightfinger_flag;

        std_msgs::Int32 leftfinger_value;
        int leftfinger_flag;

        std_msgs::Int32 rightuppump_value;
        int rightuppump_flag;

        std_msgs::Int32 rightdownpump_value;
        int rightdownpump_flag;

        std_msgs::Int32 leftuppump_value;
        int leftuppump_flag;

        std_msgs::Int32 leftdownpump_value;
        int leftdownpump_flag;



};

BallistaSerial::BallistaSerial()
{

    cmd_vel = nh.subscribe < geometry_msgs::Twist > ("/cmd_vel", 2, &BallistaSerial::cmd_vel_cb, this);
    cmd_vel_out = nh.advertise < geometry_msgs::Twist > ("/cmd_vel_out", 2);
    cmd_vel_in = nh.subscribe < geometry_msgs::Twist > ("/cmd_vel_in", 2, &BallistaSerial::cmd_vel_in_cb, this);

    enable_motors = nh.subscribe < std_msgs::Int32 > ("/enable_motors", 2, &BallistaSerial::enable_motors_cb, this);
    enable_motors_out = nh.advertise < std_msgs::Int32 > ("/enable_motors_out", 2);
    enable_motors_in = nh.subscribe < std_msgs::Int32 > ("/enable_motors_in", 2, &BallistaSerial::enable_motors_in_cb, this);

    color = nh.subscribe < std_msgs::Int32 > ("/color", 2, &BallistaSerial::color_cb, this);
    color_out = nh.advertise < std_msgs::Int32 > ("/color_out", 2);
    color_in = nh.subscribe < std_msgs::Int32 > ("/color_in", 2, &BallistaSerial::color_in_cb, this);

    alpha = nh.subscribe < std_msgs::Int32 > ("/alpha", 2, &BallistaSerial::alpha_cb, this);
    alpha_out = nh.advertise < std_msgs::Int32 > ("/alpha_out", 2);
    alpha_in = nh.subscribe < std_msgs::Int32 > ("/alpha_in", 2, &BallistaSerial::alpha_in_cb, this);

    delta = nh.subscribe < std_msgs::Int32 > ("/delta", 2, &BallistaSerial::delta_cb, this);
    delta_out = nh.advertise < std_msgs::Int32 > ("/delta_out", 2);
    delta_in = nh.subscribe < std_msgs::Int32 > ("/delta_in", 2, &BallistaSerial::delta_in_cb, this);

    ardugoal = nh.subscribe < indomptable_nav::ArduGoal > ("/ardugoal", 2, &BallistaSerial::ardugoal_cb, this);
    ardugoal_out = nh.advertise < indomptable_nav::ArduGoal > ("/ardugoal_out", 2);
    ardugoal_in = nh.subscribe < indomptable_nav::ArduGoal > ("/ardugoal_in", 2, &BallistaSerial::ardugoal_in_cb, this);

    rightfinger = nh.subscribe < std_msgs::Int32 > ("/right_finger", 2, &BallistaSerial::rightfinger_cb, this);
    rightfinger_out = nh.advertise < std_msgs::Int32 > ("/right_finger_out", 2);
    rightfinger_in = nh.subscribe < std_msgs::Int32 > ("/right_finger_in", 2, &BallistaSerial::rightfinger_in_cb, this);

    leftfinger = nh.subscribe < std_msgs::Int32 > ("/left_finger", 2, &BallistaSerial::leftfinger_cb, this);
    leftfinger_out = nh.advertise < std_msgs::Int32 > ("/left_finger_out", 2);
    leftfinger_in = nh.subscribe < std_msgs::Int32 > ("/left_finger_in", 2, &BallistaSerial::leftfinger_in_cb, this);

    rightuppump = nh.subscribe < std_msgs::Int32 > ("/right_up_pump", 2, &BallistaSerial::rightuppump_cb, this);
    rightuppump_out = nh.advertise < std_msgs::Int32 > ("/right_up_pump_out", 2);
    rightuppump_in = nh.subscribe < std_msgs::Int32 > ("/right_up_pump_in", 2, &BallistaSerial::rightuppump_in_cb, this);

    rightdownpump = nh.subscribe < std_msgs::Int32 > ("/right_down_pump", 2, &BallistaSerial::rightdownpump_cb, this);
    rightdownpump_out = nh.advertise < std_msgs::Int32 > ("/right_down_pump_out", 2);
    rightdownpump_in = nh.subscribe < std_msgs::Int32 > ("/right_down_pump_in", 2, &BallistaSerial::rightdownpump_in_cb, this);

    leftuppump = nh.subscribe < std_msgs::Int32 > ("/left_up_pump", 2, &BallistaSerial::leftuppump_cb, this);
    leftuppump_out = nh.advertise < std_msgs::Int32 > ("/left_up_pump_out", 2);
    leftuppump_in = nh.subscribe < std_msgs::Int32 > ("/left_up_pump_in", 2, &BallistaSerial::leftuppump_in_cb, this);

    leftdownpump = nh.subscribe < std_msgs::Int32 > ("/left_down_pump", 2, &BallistaSerial::leftdownpump_cb, this);
    leftdownpump_out = nh.advertise < std_msgs::Int32 > ("/left_down_pump_out", 2);
    leftdownpump_in = nh.subscribe < std_msgs::Int32 > ("/left_down_pump_in", 2, &BallistaSerial::leftdownpump_in_cb, this);



    cmd_vel_flag = 1;
    enable_motors_flag = 1;
    color_flag = 1;
    alpha_flag = 1;
    delta_flag = 1;
    ardugoal_flag = 1;
    rightfinger_flag = 1;
    leftfinger_flag = 1;
    rightuppump_flag = 1;
    rightdownpump_flag = 1;
    leftuppump_flag = 1;
    leftdownpump_flag = 1;



}

void BallistaSerial::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr & my_var)
{

    if(cmd_vel_flag == 0) {
        ROS_ERROR("Cant send data");
    cmd_vel_value.linear = my_var->linear;
    cmd_vel_value.angular = my_var->angular;

    }
    else {
    ROS_ERROR("Receiving cmd_vel : %f %f", my_var->linear.x, my_var->angular.z);
    cmd_vel_value.linear = my_var->linear;
    cmd_vel_value.angular = my_var->angular;
    cmd_vel_flag = 0;
    cmd_vel_out.publish(cmd_vel_value);

    usleep(10000);
    ros::spinOnce();
    usleep(10000);
    ros::spinOnce();
    usleep(10000);
    ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
    while( cmd_vel_flag == 0 ){
        cmd_vel_out.publish(cmd_vel_value);
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
    }

    }

}

void BallistaSerial::cmd_vel_in_cb(const geometry_msgs::Twist::ConstPtr & my_var)
{
    ROS_ERROR("Receiving cmd_vel_in : %f %f", my_var->linear.x, my_var->angular.z);
    if( ( fabs(my_var->linear.x - cmd_vel_value.linear.x) < 0.001 ) && ( fabs(my_var->angular.z - cmd_vel_value.angular.z) < 0.001) ) {
        ROS_ERROR("Receive MATCH");
        cmd_vel_flag = 1;
    }
}

void BallistaSerial::enable_motors_cb(const std_msgs::Int32::ConstPtr & my_var)
{

    if(enable_motors_flag == 0) {
        ROS_ERROR("Cant send data");
        enable_motors_value.data = my_var->data;

    }
    else {
        ROS_ERROR("Receiving enable_motors : %d", my_var->data);
        enable_motors_value.data = my_var->data;
        enable_motors_flag = 0;
        enable_motors_out.publish(enable_motors_value);

        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        while( enable_motors_flag == 0 ){
            enable_motors_out.publish(enable_motors_value);
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
        }

    }

}

void BallistaSerial::enable_motors_in_cb(const std_msgs::Int32::ConstPtr & my_var)
{
    ROS_ERROR("Receiving enable_motors_in : %d ", my_var->data);
    if( ( fabs(my_var->data - enable_motors_value.data) < 0.001 ) ) {
        ROS_ERROR("Receive MATCH");
        enable_motors_flag = 1;
    }
}

void BallistaSerial::color_cb(const std_msgs::Int32::ConstPtr & my_var)
{

    if(color_flag == 0) {
        ROS_ERROR("Cant send data");
        color_value.data = my_var->data;

    }
    else {
        ROS_ERROR("Receiving color : %d", my_var->data);
        color_value.data = my_var->data;
        color_flag = 0;
        color_out.publish(color_value);

        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        while( color_flag == 0 ){
            color_out.publish(color_value);
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
        }

    }

}

void BallistaSerial::color_in_cb(const std_msgs::Int32::ConstPtr & my_var)
{
    ROS_ERROR("Receiving color_in : %d ", my_var->data);
    if( ( fabs(my_var->data - color_value.data) < 0.001 ) ) {
        ROS_ERROR("Receive MATCH");
        color_flag = 1;
    }
}

void BallistaSerial::alpha_cb(const std_msgs::Int32::ConstPtr & my_var)
{

    if(alpha_flag == 0) {
        ROS_ERROR("Cant send data");
        alpha_value.data = my_var->data;

    }
    else {
        ROS_ERROR("Receiving alpha : %d", my_var->data);
        alpha_value.data = my_var->data;
        alpha_flag = 0;
        alpha_out.publish(alpha_value);

        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        while( alpha_flag == 0 ){
            alpha_out.publish(alpha_value);
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
        }

    }

}

void BallistaSerial::alpha_in_cb(const std_msgs::Int32::ConstPtr & my_var)
{
    ROS_ERROR("Receiving alpha_in : %d ", my_var->data);
    if( ( fabs(my_var->data - alpha_value.data) < 0.001 ) ) {
        ROS_ERROR("Receive MATCH");
        alpha_flag = 1;
    }
}

void BallistaSerial::delta_cb(const std_msgs::Int32::ConstPtr & my_var)
{

    if(delta_flag == 0) {
        ROS_ERROR("Cant send data");
        delta_value.data = my_var->data;

    }
    else {
        ROS_ERROR("Receiving delta : %d", my_var->data);
        delta_value.data = my_var->data;
        delta_flag = 0;
        delta_out.publish(delta_value);

        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        while( delta_flag == 0 ){
            delta_out.publish(delta_value);
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
        }
    }
}

void BallistaSerial::delta_in_cb(const std_msgs::Int32::ConstPtr & my_var)
{
    ROS_ERROR("Receiving delta_in : %d ", my_var->data);
    if( ( fabs(my_var->data - delta_value.data) < 0.002 ) ) {
        ROS_ERROR("Receive MATCH");
        delta_flag = 1;
    }
}

void BallistaSerial::ardugoal_cb(const indomptable_nav::ArduGoal::ConstPtr & my_var)
{

    if(ardugoal_flag == 0) {
        ROS_ERROR("Cant send data");
        ardugoal_value.x = my_var->x;
        ardugoal_value.y = my_var->y;
        ardugoal_value.theta = my_var->theta;
        ardugoal_value.last = my_var->last;

    }
    else {
        ROS_ERROR("Receiving ardugoal : %f %f", my_var->x, my_var->y);
        ardugoal_value.x = my_var->x;
        ardugoal_value.y = my_var->y;
        ardugoal_value.theta = my_var->theta;
        ardugoal_value.last = my_var->last;
        ardugoal_flag = 0;
        ardugoal_out.publish(ardugoal_value);

        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        while( ardugoal_flag == 0 ){
            ardugoal_out.publish(ardugoal_value);
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
        }

    }

}

void BallistaSerial::ardugoal_in_cb(const indomptable_nav::ArduGoal::ConstPtr & my_var)
{
    ROS_ERROR("Receiving ardugoal_in : %f %f %f", my_var->x, my_var->y, my_var->theta);
    if( ( fabs(my_var->x - ardugoal_value.x) < 0.002 ) && ( fabs(my_var->y - ardugoal_value.y) < 0.002 ) && ( fabs(my_var->theta - ardugoal_value.theta) < 0.002 ) ) {
        ROS_ERROR("Receive MATCH");
        ardugoal_flag = 1;
    }
}


void BallistaSerial::rightfinger_cb(const std_msgs::Int32::ConstPtr & my_var)
{

    if(rightfinger_flag == 0) {
        ROS_ERROR("Cant send data");
        rightfinger_value.data = my_var->data;

    }
    else {
        ROS_ERROR("Receiving rightfinger : %d", my_var->data);
        rightfinger_value.data = my_var->data;
        rightfinger_flag = 0;
        rightfinger_out.publish(rightfinger_value);

        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        while( rightfinger_flag == 0 ){
            rightfinger_out.publish(rightfinger_value);
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
        }
    }
}   

void BallistaSerial::rightfinger_in_cb(const std_msgs::Int32::ConstPtr & my_var)
{
    ROS_ERROR("Receiving rightfinger_in : %d ", my_var->data);
    if( ( fabs(my_var->data - rightfinger_value.data) < 0.002 ) ) {
        ROS_ERROR("Receive MATCH");
        rightfinger_flag = 1;
    }
}

void BallistaSerial::leftfinger_cb(const std_msgs::Int32::ConstPtr & my_var)
{

    if(leftfinger_flag == 0) {
        ROS_ERROR("Cant send data");
        leftfinger_value.data = my_var->data;

    }
    else {
        ROS_ERROR("Receiving leftfinger : %d", my_var->data);
        leftfinger_value.data = my_var->data;
        leftfinger_flag = 0;
        leftfinger_out.publish(leftfinger_value);

        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        while( leftfinger_flag == 0 ){
            leftfinger_out.publish(leftfinger_value);
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
        }
    }
}   

void BallistaSerial::leftfinger_in_cb(const std_msgs::Int32::ConstPtr & my_var)
{
    ROS_ERROR("Receiving leftfinger_in : %d ", my_var->data);
    if( ( fabs(my_var->data - leftfinger_value.data) < 0.002 ) ) {
        ROS_ERROR("Receive MATCH");
        leftfinger_flag = 1;
    }
}

void BallistaSerial::rightuppump_cb(const std_msgs::Int32::ConstPtr & my_var)
{

    if(rightuppump_flag == 0) {
        ROS_ERROR("Cant send data");
        rightuppump_value.data = my_var->data;

    }
    else {
        ROS_ERROR("Receiving rightuppump : %d", my_var->data);
        rightuppump_value.data = my_var->data;
        rightuppump_flag = 0;
        rightuppump_out.publish(rightuppump_value);

        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        while( rightuppump_flag == 0 ){
            rightuppump_out.publish(rightuppump_value);
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
        }
    }
}   

void BallistaSerial::rightuppump_in_cb(const std_msgs::Int32::ConstPtr & my_var)
{
    ROS_ERROR("Receiving rightuppump_in : %d ", my_var->data);
    if( ( fabs(my_var->data - rightuppump_value.data) < 0.002 ) ) {
        ROS_ERROR("Receive MATCH");
        rightuppump_flag = 1;
    }
}

void BallistaSerial::rightdownpump_cb(const std_msgs::Int32::ConstPtr & my_var)
{

    if(rightdownpump_flag == 0) {
        ROS_ERROR("Cant send data");
        rightdownpump_value.data = my_var->data;

    }
    else {
        ROS_ERROR("Receiving rightdownpump : %d", my_var->data);
        rightdownpump_value.data = my_var->data;
        rightdownpump_flag = 0;
        rightdownpump_out.publish(rightdownpump_value);

        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        while( rightdownpump_flag == 0 ){
            rightdownpump_out.publish(rightdownpump_value);
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
        }
    }
}   

void BallistaSerial::rightdownpump_in_cb(const std_msgs::Int32::ConstPtr & my_var)
{
    ROS_ERROR("Receiving rightdownpump_in : %d ", my_var->data);
    if( ( fabs(my_var->data - rightdownpump_value.data) < 0.002 ) ) {
        ROS_ERROR("Receive MATCH");
        rightdownpump_flag = 1;
    }
}

void BallistaSerial::leftuppump_cb(const std_msgs::Int32::ConstPtr & my_var)
{

    if(leftuppump_flag == 0) {
        ROS_ERROR("Cant send data");
        leftuppump_value.data = my_var->data;

    }
    else {
        ROS_ERROR("Receiving leftuppump : %d", my_var->data);
        leftuppump_value.data = my_var->data;
        leftuppump_flag = 0;
        leftuppump_out.publish(leftuppump_value);

        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        while( leftuppump_flag == 0 ){
            leftuppump_out.publish(leftuppump_value);
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
        }
    }
}   

void BallistaSerial::leftuppump_in_cb(const std_msgs::Int32::ConstPtr & my_var)
{
    ROS_ERROR("Receiving leftuppump_in : %d ", my_var->data);
    if( ( fabs(my_var->data - leftuppump_value.data) < 0.002 ) ) {
        ROS_ERROR("Receive MATCH");
        leftuppump_flag = 1;
    }
}

void BallistaSerial::leftdownpump_cb(const std_msgs::Int32::ConstPtr & my_var)
{

    if(leftdownpump_flag == 0) {
        ROS_ERROR("Cant send data");
        leftdownpump_value.data = my_var->data;

    }
    else {
        ROS_ERROR("Receiving leftdownpump : %d", my_var->data);
        leftdownpump_value.data = my_var->data;
        leftdownpump_flag = 0;
        leftdownpump_out.publish(leftdownpump_value);

        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        usleep(10000);
        ros::spinOnce();
        while( leftdownpump_flag == 0 ){
            leftdownpump_out.publish(leftdownpump_value);
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
            usleep(10000);
            ros::spinOnce();
        }
    }
}   

void BallistaSerial::leftdownpump_in_cb(const std_msgs::Int32::ConstPtr & my_var)
{
    ROS_ERROR("Receiving leftdownpump_in : %d ", my_var->data);
    if( ( fabs(my_var->data - leftdownpump_value.data) < 0.002 ) ) {
        ROS_ERROR("Receive MATCH");
        leftdownpump_flag = 1;
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
    ros::init(argc, argv, "Ballista_Serial");
    BallistaSerial ballistaserial;

    // Refresh rate
    ros::Rate loop_rate(25);
    float rotation = 0.0;
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Duration(1.0).sleep();
    ros::shutdown();
}



