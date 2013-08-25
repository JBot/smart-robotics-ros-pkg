/* 
 * rosserial Time and TF Example
 * Publishes a transform at current time
 */

/* INCLUDES */
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>

#include "DualVNH5019MotorShield.h"



/* DEFINES */
#define ADCVALUE 0.001256637



/* VARIABLES */
DualVNH5019MotorShield md;

ros::NodeHandle  nh;

//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;

char child_link[] = "/base_link";
char parent_link[] = "/odom";

volatile double shoulder_des = 0;
volatile double elbow_des = 0;
volatile double error_prev_shoulder = 0;
volatile double error_sum_shoulder = 0;
volatile double error_prev_elbow = 0;
volatile double error_sum_elbow = 0;
volatile double PID_shoulder = 0;
volatile double PID_elbow = 0;
volatile int sensorValuePosShoulder = 0;
volatile int sensorValuePosElbow = 0;
const int analogInPinShoulder = A15;  // Analog input pin that the potentiometer is attached to
const int analogInPinElbow = A14;  // Analog input pin that the potentiometer is attached to

char *as[] = {"shoulder_pitch_joint", "elbow_pitch_joint"};
float positions_tab[2];
float velocities_tab[2];
float efforts_tab[2];


/* PUBLISHERS */
/*
sensor_msgs::JointState joints;
ros::Publisher joint_state_publisher("arm/jointstate_arduino", &joints);*/

std_msgs::Float32 shoulder_feedback;
ros::Publisher pub_shoulder("shoulder_pitch_state", &shoulder_feedback);

std_msgs::Float32 elbow_feedback;
ros::Publisher pub_elbow("elbow_pitch_state", &elbow_feedback);



/* CALLBACKS */

void messageCbShoulder( const std_msgs::Float32& toggle_msg){
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
	shoulder_des = toggle_msg.data;
}

ros::Subscriber<std_msgs::Float32> sub_shoulder("shoulder_pitch_joint", messageCbShoulder );

void messageCbElbow( const std_msgs::Float32& toggle_msg){
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
	elbow_des = toggle_msg.data;
}

ros::Subscriber<std_msgs::Float32> sub_elbow("elbow_pitch_joint", messageCbElbow );



/* FUNCTIONS */

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}
/*
geometry_msgs::Quaternion createQuaternion(double yaw, double pitch, double roll)
{
  geometry_msgs::Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = sin(yaw * 0.5);
  q.w = cos(yaw * 0.5);
  //return q;
}
*/
void read_positions(void)
{
  sensorValuePosShoulder = analogRead(analogInPinShoulder);
  sensorValuePosElbow = analogRead(analogInPinElbow);
}

double convert_adc(int value)
{
  return -(value - 450) * ADCVALUE;
}

void compute_PIDs(double shoulder, double elbow)
{
  
  double errorShoulder =  convert_adc(sensorValuePosShoulder) - shoulder;
  double errorElbow =  convert_adc(sensorValuePosElbow) - elbow;
  
  error_sum_shoulder += errorShoulder;
  error_sum_elbow += errorElbow;
  
  if(error_sum_shoulder > (50* ADCVALUE))
    error_sum_shoulder = 50* ADCVALUE;
  if(error_sum_shoulder < (-50* ADCVALUE))
    error_sum_shoulder = -50* ADCVALUE;
  if(error_sum_elbow > (50* ADCVALUE))
    error_sum_elbow = 50* ADCVALUE;
  if(error_sum_elbow < (-50* ADCVALUE))
    error_sum_elbow = -50* ADCVALUE;
  
  PID_shoulder = 5000 * errorShoulder + 600 * error_prev_shoulder + 50 * error_sum_shoulder; // un peu mou
  PID_elbow = 10000 * errorElbow + 1000 * error_prev_elbow + 200 * error_sum_elbow;
 
  error_prev_shoulder = errorShoulder; 
  error_prev_elbow = errorElbow; 
  
  if(PID_shoulder > 400) 
    PID_shoulder = 400;
  if(PID_shoulder < -400)
    PID_shoulder = -400;
  if(PID_elbow > 400) 
    PID_elbow = 400;
  if(PID_elbow < -400)
    PID_elbow = -400;
 /*
  Serial.print("PID : ");
  Serial.print(PID);
  Serial.print(" ERREUR : ");
  Serial.print(erreur);
  Serial.print(" SENSOR : ");
  Serial.println(sensorValue);
  */
 
  
  
  
  
}

void move_motors(void)
{
  md.setM1Speed(PID_shoulder);
  stopIfFault();
  /*
  md.setM1M2Speed(PID_shoulder, PID_elbow);
  stopIfFault();
  */
}




void setup()
{
  md.init();
  analogReference(EXTERNAL);
  
  nh.initNode();
  //broadcaster.init(nh);
  //nh.advertise(joint_state_publisher);
  nh.advertise(pub_shoulder);
  nh.advertise(pub_elbow);
  nh.subscribe(sub_shoulder);
  nh.subscribe(sub_elbow);
  /*
  joints.name_length = 2;
  joints.position_length = 2;
  joints.velocity_length = 2;
  joints.effort_length = 2;*/
  /*joints.name[2];
  joints.position[2];*/
  /*
  joints.name = as;
  positions_tab[0] = 0.0;
  positions_tab[1] = 0.0;
  velocities_tab[0] = 0.0;
  velocities_tab[1] = 0.0;
  efforts_tab[0] = 0.0;
  efforts_tab[1] = 0.0;*/
}

void loop()
{  

  read_positions();            
    
  compute_PIDs(shoulder_des, elbow_des);

  move_motors();    
 /* 
  joints.header.stamp = nh.now();
  positions_tab[0] = convert_adc(sensorValuePosShoulder);
  positions_tab[1] = convert_adc(sensorValuePosElbow);
  joints.position = positions_tab;
  joints.velocity = velocities_tab;
  joints.effort = efforts_tab;
  
  joint_state_publisher.publish( &joints );
 */ 

  shoulder_feedback.data = convert_adc(sensorValuePosShoulder);
  elbow_feedback.data = convert_adc(sensorValuePosElbow);  
  
  pub_shoulder.publish( &shoulder_feedback );
  pub_elbow.publish( &elbow_feedback );
  
/*
  t.header.frame_id = parent_link;
  t.child_frame_id = child_link;
  t.transform.translation.x = 1.0; 
  t.transform.translation.y = 1.0; 
  t.transform.translation.z = 1.0; 

  t.transform.rotation.x = 0;
  t.transform.rotation.y = 0;
  t.transform.rotation.z = sin(yaw * 0.5);
  t.transform.rotation.w = cos(yaw * 0.5);
  t.transform.rotation = tf::createQuaternionFromYaw(1.57);
  
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0; 
  t.transform.rotation.z = 0.0; 
  t.transform.rotation.w = 1.0;  
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);*/
  nh.spinOnce();
  
  delay(10);
}
