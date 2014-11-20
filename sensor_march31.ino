/*
***Sensor code***
Reads "1" when nothing is detected
Reads "0" when an object is detected
Has a distance of up to 10 centimeters*/

#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <Servo.h>
#include <EEPROM.h>
#include <RobotOpen.h>

int sensor1 = 3; //set up DIGITAL pin for sensor
int sensor2 = 4; //set up DIGITAL pin for sensor
int sensor3 = 5; //set up DIGITAL pin for sensor
int sensor4 = 6; //set up DIGITAL pin for sensor
int detection; //sensor variable

void setup(){
  RobotOpen.begin(&enabled, &disabled, &timedtasks); //communicate with RobotOpen
  pinMode(sensor1, INPUT); //set sensor pin as INPUT
  pinMode(sensor2, INPUT); //set sensor pin as INPUT
  pinMode(sensor3, INPUT); //set sensor pin as INPUT
  pinMode(sensor4, INPUT); //set sensor pin as INPUT
  Serial.begin(9600); //serial communication
}

void enabled(){
  Serial.println(digitalRead(sensor1)); //Print sensor to serial
  Serial.println(digitalRead(sensor2)); //Print sensor to serial
  Serial.println(digitalRead(sensor3)); //Print sensor to serial
  Serial.println(digitalRead(sensor4)); //Print sensor to serial
  delay(300); //delay time
  
  detection = digitalRead(sensor1) + digitalRead(sensor2) + digitalRead(sensor3) + digitalRead(sensor4);
  Serial.println(detection); //Print detection number to serial - it should be 4 if nothing is detected by any sensor

  //This should disable the rover if any sensor detects an object
  if (detection < 4){
     //state = Off
  }
}

void timedtasks() {  
  //Publish to RobotOpen
  RODashboard.publish("Uptime Seconds", ROStatus.uptimeSeconds());
  RODashboard.publish("Enabled state", ROStatus.isEnabled());
  RODashboard.publish("Sensor1", digitalRead(sensor1));
  RODashboard.publish("Sensor2", digitalRead(sensor2));
  RODashboard.publish("Sensor3", digitalRead(sensor3));
  RODashboard.publish("Sensor4", digitalRead(sensor4));
}
void disabled() {
  // safety code
}
// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}
