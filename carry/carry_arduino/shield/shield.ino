/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIXELS_PIN 6
#define RELAY_PIN 5
#define VOLTAGE_PIN 0

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIXELS_PIN, NEO_GRB + NEO_KHZ800);

ros::NodeHandle  nh;

volatile int recharge_state;
volatile unsigned long last_time_output = 0;

volatile unsigned long last_time_sensing = 0;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  recharge_state = 1;
  last_time_output = millis();
}
ros::Subscriber<std_msgs::Empty> sub("recharge_battery", messageCb );

std_msgs::Float32 float_msg;
ros::Publisher battery_pub("battery_level", &float_msg);


void setup()
{
  pinMode(13, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  recharge_state = 0;
  
  last_time_sensing= millis();
  
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  nh.initNode();
  nh.advertise(battery_pub);
  nh.subscribe(sub);
}

void loop()
{
  
  nh.spinOnce();
  delay(50);
  
  if( abs(last_time_sensing*1.0 - millis()*1.0) > (30.0*1000.0) )
  {
    last_time_sensing = millis();
    float_msg.data = analogRead(VOLTAGE_PIN) * 5.0 / 1023.0;
    battery_pub.publish( &float_msg );
  }
  
  switch(recharge_state)
  {
    case 0 :
      break;
    case 1 :
      digitalWrite(RELAY_PIN, HIGH);
      if( abs(last_time_output*1.0 - millis()*1.0) > (3.0*1000.0) )
      {
        digitalWrite(RELAY_PIN, LOW);
        last_time_output = millis();
        recharge_state = 2;
      }
      break;
    case 2 :
      if( abs(last_time_output*1.0 - millis()*1.0) > (2.0*1000.0) )
      {
        digitalWrite(RELAY_PIN, HIGH);
        last_time_output = millis();
        recharge_state = 3;
      }
      break;
    case 3 :
      if( abs(last_time_output*1.0 - millis()*1.0) > (0.5*1000.0) )
      {
        digitalWrite(RELAY_PIN, LOW);
        last_time_output = millis();
        recharge_state = 0;
      }
      break;
    default :
      recharge_state = 0;
      break;
  }
}
