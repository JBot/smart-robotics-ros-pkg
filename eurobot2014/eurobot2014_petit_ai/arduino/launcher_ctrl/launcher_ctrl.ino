/*
  Mega multple serial test
 
 Receives from the main serial port, sends to the others. 
 Receives from serial port 1, sends to the main serial (Serial 0).
 
 This example works only on the Arduino Mega
 
 The circuit: 
 * Any serial device attached to Serial port 1
 * Serial monitor open on Serial port 0:
 
 created 30 Dec. 2008
 by Tom Igoe
 
 This example code is in the public domain.
 
 */

#include <Servo.h> 

Servo myservo;
Servo mybrushless;

int brushvalue = 20;

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  //Serial1.begin(9600);
  myservo.attach(3);
  mybrushless.attach(50);
  
  myservo.write(75);
  mybrushless.write(brushvalue);
  
}

void loop() {
  // read from port 1, send to port 0:
  if (Serial.available()) {
    int inByte = Serial.read();
    //Serial.write(inByte);
 
   if(inByte == 43){ // +
     //brushvalue = brushvalue + 5;
     brushvalue = 82;
     mybrushless.write(brushvalue);
     Serial.println(brushvalue);
   }
   if(inByte == 45){ // -
     //brushvalue = brushvalue - 5;
     brushvalue = 20;
     mybrushless.write(brushvalue);
     Serial.println(brushvalue);
   }
   if(inByte == 111){ // o
     openclose();
   }
   if(inByte == 112){ // p
     myservo.write(110);;
   }
   if(inByte == 99){ // c
     myservo.write(75);
   }
   
  }
}

void openclose(void)
{
  myservo.write(110);
  //delay(280);
  delay(500);
  myservo.write(75);
}
