/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;
Servo servo8;
Servo servo9;
Servo servo10;
Servo servo11;
Servo servo12;
Servo servo13;
Servo servo14;
Servo servo15;
Servo servo16;

void servo1_cb( const std_msgs::UInt16& cmd_msg){
  servo1.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo2_cb( const std_msgs::UInt16& cmd_msg){
  servo2.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo3_cb( const std_msgs::UInt16& cmd_msg){
  servo3.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo4_cb( const std_msgs::UInt16& cmd_msg){
  servo4.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo5_cb( const std_msgs::UInt16& cmd_msg){
  servo5.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo6_cb( const std_msgs::UInt16& cmd_msg){
  servo6.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo7_cb( const std_msgs::UInt16& cmd_msg){
  servo7.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo8_cb( const std_msgs::UInt16& cmd_msg){
  servo8.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo9_cb( const std_msgs::UInt16& cmd_msg){
  servo9.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo10_cb( const std_msgs::UInt16& cmd_msg){
  servo10.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo11_cb( const std_msgs::UInt16& cmd_msg){
  servo11.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo12_cb( const std_msgs::UInt16& cmd_msg){
  servo12.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo13_cb( const std_msgs::UInt16& cmd_msg){
  servo13.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo14_cb( const std_msgs::UInt16& cmd_msg){
  servo14.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo15_cb( const std_msgs::UInt16& cmd_msg){
  servo15.write(cmd_msg.data); //set servo angle, should be from 0-180  
}
void servo16_cb( const std_msgs::UInt16& cmd_msg){
  servo16.write(cmd_msg.data); //set servo angle, should be from 0-180  
}


ros::Subscriber<std_msgs::UInt16> sub1("servo1", servo1_cb);
ros::Subscriber<std_msgs::UInt16> sub2("servo2", servo2_cb);
ros::Subscriber<std_msgs::UInt16> sub3("servo3", servo3_cb);
ros::Subscriber<std_msgs::UInt16> sub4("servo4", servo4_cb);
ros::Subscriber<std_msgs::UInt16> sub5("servo5", servo5_cb);
ros::Subscriber<std_msgs::UInt16> sub6("servo6", servo6_cb);
ros::Subscriber<std_msgs::UInt16> sub7("servo7", servo7_cb);
ros::Subscriber<std_msgs::UInt16> sub8("servo8", servo8_cb);
ros::Subscriber<std_msgs::UInt16> sub9("servo9", servo9_cb);
ros::Subscriber<std_msgs::UInt16> sub10("servo10", servo10_cb);
ros::Subscriber<std_msgs::UInt16> sub11("servo11", servo11_cb);
ros::Subscriber<std_msgs::UInt16> sub12("servo12", servo12_cb);
ros::Subscriber<std_msgs::UInt16> sub13("servo13", servo13_cb);
ros::Subscriber<std_msgs::UInt16> sub14("servo14", servo14_cb);
ros::Subscriber<std_msgs::UInt16> sub15("servo15", servo15_cb);
ros::Subscriber<std_msgs::UInt16> sub16("servo16", servo16_cb);

void setup(){
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.subscribe(sub6);
  nh.subscribe(sub7);
  nh.subscribe(sub8);
  nh.subscribe(sub9);
  nh.subscribe(sub10);
  nh.subscribe(sub11);
  nh.subscribe(sub12);
  nh.subscribe(sub13);
  nh.subscribe(sub14);
  nh.subscribe(sub15);
  nh.subscribe(sub16);
  
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
  servo5.attach(6);
  servo6.attach(7);
  servo7.attach(8);
  servo8.attach(9);
  servo9.attach(10);
  servo10.attach(11);
  servo11.attach(12);
  servo12.attach(13);
  servo13.attach(14);
  servo14.attach(15);
  servo15.attach(16);
  servo16.attach(17);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
