#include <ros/ros.h>
//#include <joy/Joy.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


class TeleopNeato
{
public:
  TeleopNeato();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  int button_prev_status;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopNeato::TeleopNeato():
  linear_(1),
  angular_(2),
  a_scale_(0.3), // 0.2 // 0.6
  l_scale_(0.1) // 0.18 // 0.2
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  button_prev_status = 0;

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopNeato::joyCallback, this);

}

void TeleopNeato::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = a_scale_*joy->axes[angular_];
  vel.angular.y = a_scale_*joy->axes[3];
  vel.angular.x = 0;
  vel.linear.x = l_scale_*joy->axes[linear_];
  vel.linear.y = l_scale_*joy->axes[0];
  vel.linear.z = 0;

  vel_pub_.publish(vel);

  if(joy->buttons[0] == 1) {
    if( button_prev_status == 0) {
      // Power on the vacuum

    }

  }
  button_prev_status = joy->buttons[0];
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");
  TeleopNeato teleop_neato;

  ros::spin();
}


