//Code for Purdue EPICS - AAEE Mars Rover
//Authors: Tom Stepp and Sujit Shivaprasad
//The goal is to control a rover similar to NASA's curiosity rover
//This includes code for drive modes (motors and servos), sensors, video, and a robotic arm.

//Libraries
#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>
#include <Servo.h>
#include <EEPROM.h>.
#include <RobotOpen.h>
#include <RODashboard.h>
#include <ROJoystick.h>
#include <ROPWM.h>
#include <ROStatus.h>
#include <ROTimer.h>

// Input/Output setup
ROJoystick usb1(1); //Enables input from joystick
//RODigitalIO sensor1(3, INPUT); //Sensor setup
//RODigitalIO sensor2(4, INPUT);
//RODigitalIO sensor3(5, INPUT);
//RODigitalIO sensor4(6, INPUT);
ROPWM motorBL(0); //Enables back left motor
ROPWM motorFL(1); //Enables front left motor
ROPWM motorBR(2); //Enables back right motor
ROPWM motorFR(3); //Enables front right motor
ROPWM motorMR(4); //Enables middle right
ROPWM motorML(5); //Enables middle left
ROPWM servoFL(7);  //Enables front left servo
ROPWM servoFR(8);  //Enables front right servo
//ROPWM servoBL(9); //Enables back left servo
//ROPWM servoBR(10); //Enables back right servo
ROPWM servo_lr(7); //Camera servo (left/right)
ROPWM servo_ud(8); //Camera servo (up/down)

//State Machine Variables
int state = 2; //controls state machine
#define ForwardDrive   0
#define Drive360       1
#define Off            2
#define TankDrive      3
#define Arm            4
int a = 0; //Control variable for button A --Drive360 mode
int x = 0; //Control variable for button X --ForwardDrive
int b = 0; //Control variable for button B --Off
int y = 0; //Control variable for button Y --Arm
int r_trig = 0; //Control for right trigger --TankDrive
int num = 1; //Control variable for switch state: only runs that code when a button resets "num" to zero
int angle_ud = 0; //Initial angle for up/down camera servo
ROTimer repeatingloop1; //Time delay for up/down servo

//Sensor Variables
//int detection; //Control variable for sensor detection
//int sensor_num = 4; //Number of sensors
//#define Sensor_Num     4

void setup()
{
  RobotOpen.begin(&enabled, &disabled, &timedtasks); //Initiate communication
  Serial.begin(9600); // initialize the serial communication
  repeatingloop1.queue(0); //Used as delay for up/down camera servo
}

/* This is your primary robot loop - all of your code should live here that allows the robot to operate*/
void enabled() 
{  
  /// Sensor Code ///
//  detection = sensor1.read() + sensor2.read() + sensor3.read() + sensor4.read();
//  if (detection < Sensor_Num){
//    state = Off;
//    num = 0;
//    a = 0;
//    x = 0;
//    b = 0;
//    y = 0;
//    r_trig = 0; 
//  }
    
    /// Camera Code ///
    if (usb1.dPadRight() ==0 && usb1.dPadLeft() == 0 && usb1.dPadUp() ==0 && usb1.dPadDown() == 0)
    {
      RobotOpen.detachPWM(7) ; //detach servo
    }
    if (usb1.dPadRight() == 1)
    {
      RobotOpen.attachPWM(7);//attach servo
      servo_lr.write(105); //Turns camera to the right
    }
    if (usb1.dPadLeft() == 1)
    {
      RobotOpen.attachPWM(7);//attach servo
      servo_lr.write(97); // Turns camera to the left
    }
    if (repeatingloop1.ready())
    {
    //Camera servos (up/down)
    if (usb1.dPadUp() == true && angle_ud < 180)
    {
      RobotOpen.detachPWM(7);//Make sure left/right servo is detached
      angle_ud = angle_ud + 2; // Add 2 degrees to angle
      servo_ud.write(angle_ud); //Moves camera up
    }
    if (usb1.dPadDown() == true && angle_ud > 0)
    {
      RobotOpen.detachPWM(7);//Make sure left/right servo is detached
      angle_ud = angle_ud - 2; //Subtract 2 degrees to angle
      servo_ud.write(angle_ud); // Moves camera down
    }
    repeatingloop1.queue(60); //Time delay is 60 ms
    }
  
  /// Drive Modes ///
  //Determine if machine state should be "Drive360"
  if (usb1.btnA()){
    if(a == 0){
      a = 1;
      x = 0;
      b = 0;
      y = 0;
      r_trig = 0; }
  }
  if(a == 1){
     state = Drive360;
     num = 0;
  }
    
  //Determine if machine state should be "ForwardDrive"
  if (usb1.btnX()) {   
    if(x == 0){
      x = 1;
      a = 0;
      b = 0;
      r_trig = 0;
      y = 0;}
  }
  if(x == 1){
      state = ForwardDrive;
      num = 0; 
  }
  
  //Set machine state to "Off" 
  if (usb1.btnB()) {   
    if(b == 0){
      b = 1;
      a = 0;
      x = 0;
      r_trig = 0;
      y = 0;}
  }
  if(b == 1){
      state = Off;
      num = 0; 
  }
  //Set machine state to "TankDrive"
   if (usb1.rTrigger()) {   
    if(r_trig == 0){
      r_trig = 1;
      b = 0;
      a = 0;
      x = 0;
      y = 0;}
  }
  if(r_trig == 1){
      state = TankDrive;
      num = 0; 
  }
  
  if (num < 1)
  {
  //Call code for a specific state
  switch(state) {
    case ForwardDrive: DoForwardDrive(); break;
    case Drive360: DoDrive360(); break;
    case Off: DoOff(); break;
    case TankDrive: DoTankDrive(); break;
  }
  }
}

//Machine drive state functions//

//Forward Drive Mode: set servos to 90 degrees. Map all motors to Y-direction of left joystick.
void DoForwardDrive () {
  servoFL.write(90); //Set servo to mid-point
  servoFR.write(90);
//  servoBL.write(90); 
//  servoBR.write(90);
  motorBR.write(usb1.leftY()); 
  motorFR.write(usb1.leftY()); 
  motorBL.write(usb1.leftY()); 
  motorFL.write(usb1.leftY()); 
  motorML.write(usb1.leftY()); 
  motorMR.write(usb1.leftY()); 
  num++;
}

//Tank Drive Mode: set servos to 90 degrees. Map left motors to left joystick and right motors to right joystick
void DoTankDrive () {
  servoFL.write(90); 
  servoFR.write(90);
//  servoBL.write(90); 
//  servoBR.write(90);
  motorBR.write(usb1.rightY()); 
  motorFR.write(usb1.rightY()); 
  motorBL.write(usb1.leftY()); 
  motorFL.write(usb1.leftY()); 
  motorML.write(usb1.leftY()); 
  motorMR.write(usb1.rightY()); 
  num++;
}

//360 turn Mode: set servos to correct angle and map motors to X-direction of left joystick
void DoDrive360 () {
  servoFL.write(180);
  servoFR.write(180);
//  servoBL.write(180);
//  servoBR.write(180);
  motorBR.write(usb1.leftX()); 
  motorFR.write(usb1.leftX()); 
  motorBL.write(usb1.leftX()); 
  motorFL.write(usb1.leftX()); 
  motorMR.write(usb1.leftX()); 
  motorML.write(usb1.leftX()); 
  num++;
}

//Off Mode: calls "disabled" to automatically disable all outputs
void DoOff () {
  disabled();
  num++;
}

/* This is called while the robot is disabled
 * All outputs are automatically disabled (PWM, Solenoid, Digital Outs)*/
void disabled() {
  // safety code
}

/* This loop ALWAYS runs - only place code here that can run during a disabled state
 * This is also a good spot to put driver station publish code*/
void timedtasks() {  
  RODashboard.publish("Uptime Seconds", ROStatus.uptimeSeconds());
  RODashboard.publish("Enabled state", ROStatus.isEnabled());
  
  //Joystick input to the Dashboard
  RODashboard.publish("Left Y Joystick Input", usb1.leftY());
  RODashboard.publish("Right Y Joystick Input", usb1.rightY());
  RODashboard.publish("Left X Joystick Input", usb1.leftX());
  RODashboard.publish("Right X Joystick Input", usb1.rightX());
  RODashboard.publish("A button Input", usb1.btnA());
  RODashboard.publish("B button Input", usb1.btnB());
  RODashboard.publish("X button Input", usb1.btnX());
  RODashboard.publish("Y button Input", usb1.btnY());                   
  
  //Sensor input
//  RODashboard.publish("Sensor1", read(sensor1));
//  RODashboard.publish("Sensor2", read(sensor2));
//  RODashboard.publish("Sensor3", read(sensor3));
//  RODashboard.publish("Sensor4", read(sensor4));
}

// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}
