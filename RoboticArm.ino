#include <Servo.h> 
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include <SoftwareSerial.h>

Servo ShoulderRot;
Servo ShoulderFlex;
Servo ElbowFlex;
Servo WristFlex;
Servo WristRot;
Servo Gripper;

SoftwareSerial testCoor(2, 4);  // pin 2 RX     pin 4 TX
int sRot = 90;    //angle at which the base is rotated
int sFlex = 135;   //angle at which the shoulder is flexed
int eFlex = 155;   //angle at which the elbow is flexed
int wrFlex = 80;  //angle at which the wrist is flexed
int wrRot = 90;   //angle at which the wrist is rotated
int grip = 150;    //gripper opening variable
int theta = 65;  // angle between ground and gripper opening
int x = 95; // x coordinate for zeroing the arm
int y = 40; // y coordinate for zeroing the arm

float const HT = 12.875; //height of the wrist flex at home position in inches
float const GLOFST = 5.6; // offset for the gripper length
float const UA = 5.75; // length of upper arm
float const FA = 7.25; // length of forearm
float const CAMOFST = 1.75; // offset for camera
float a;  // the horizontal distance from wrist flex joint to rock
float f;  // the horizontal distance from shoulder joint to wrist
float h;  // hypotenuse of the shoulder-wrist triangle, used to find g
float g;  // the vert distance from shoulder to wrist joint
float D;  // the distance the shoulder needs to be from the wrist in order to pick up rock

int const HsROT = 90; // home pos angle for base rotation
int const HsFLEX = 135; // Home pos angle for shoulder
int const HeFLEX = 155; // Home pos angle for elbow
int const HwrFLEX = 70; // Home pos angle for wrist flex
int const HwrROT = 0; // Home pos angle for wrist rotate (perpindicular to ground)
int const HGRIP = 110; // Home pos opening for gripper (closed partially)
float const REACH = 13.5; // Maximum reach for the arm
unsigned long const HrtBtInterval = 2000; // beacon interval
int const xyERR = 5;  // gives arm room for error when zeroing in on camera coordinates
int const xZeroed = 165; // the camera points at rock if x = 160
int const yZeroed = 100; // the camera points at rock if y = 100

int command_state = 0;   // state command received from Due
int homeflag = 0; // flag for going to home position
int coordinates = 0;  // zeroing flag
int getrock = 0;
int wrongrock = 0;
int saverock = 0;
int sRotNew;    //angle at which the base needs to be rotated
int sFlexNew;   //angle at which the shoulder needs to be flexed
int eFlexNew;   //angle at which the elbow needs to be flexed
int wrFlexNew;  //angle at which the wrist needs to be flexed
int wrRotNew;   //angle at which the wrist needs to be rotated
int gripNew;    //value at which the gripper needs to be opened/closed to
unsigned long nextHeartBeatms = 0;
int cur_state = 0;  // state the arm thinks it is in when it sends beacon message

String firstValue;
String secondValue;
int myString;

String inputString = "";   // input from camera used by Serial Event ISR
String coordinateString = "";  // the string to be parsed in ParseString
boolean stringComplete = true;
boolean ignoreCoords = false;

void setup() 
{ 
  Serial.begin(9600);
  testCoor.begin(9600);
  ShoulderRot.attach(11); // attaches the servos to their pins
  ShoulderFlex.attach(10);
  ElbowFlex.attach(9);
  WristFlex.attach(6);
  WristRot.attach(5);
  Gripper.attach(3);
  nextHeartBeatms = millis() + HrtBtInterval;
  placerock();
  inputString.reserve(20);
  coordinateString.reserve(20);
} 


void loop()
{
  readMessage();  // calls function to read which state I need to be in
  if (homeflag)
  {
    homepos();   // this function goes to home position 
  }
  else if (coordinates)
  {
    xyZero();    // need to figure out struct coord to read the xy-coordinates 
    //then relocating should be simple
  }
  if (getrock)
  {
    Serial.println("getrock = 1");
    pickup();  // need to create a fxn to determine if rock is in range
    // does the trig for and puts arm in position to grip the rock
    // also sends the arm home after picking up the rock
  }
  else if (wrongrock)
  {
    droprock();  // opens the gripper to drop the rock then goes home again
  }
  else if (saverock)
  {
    placerock(); // places rock 'behind' arm into basket and goes home again
  }
  beacon();                                             
  //delay(2000);
}

void beacon()
{

  if (nextHeartBeatms <= millis())
  {
    //Serial.print("Time: ");
    //Serial.println(millis());
    //Serial.print("\t");
    //Serial.println(state);
    if (cur_state == 72)
    {
      testCoor.write("H");   // arm is in the home position
      Serial.write("H");
    }
    else if (cur_state == 67)  // 'C'  arm is zeroing in on the rock
    {
      testCoor.write("A");
      Serial.write("C");
    }
    else if (cur_state == 71)  // 'G' arm is moving to pick up the rock
    {
      testCoor.write("A");
      Serial.println("G");
    }
    else if (cur_state == 83)  // 'S' arm is moving to save the rock
    {
      testCoor.write("A");
      Serial.println("S");
    }
    else if (cur_state == 78)  // 'N' arm is moving to drop the rock
    {
      testCoor.write("A");
      Serial.println("N");
    }
    else if (cur_state == 1)   // the arm is moving home
    {
      testCoor.write("A");
      Serial.println("A");
    }
    else if (cur_state == 2)   // Means the rock is out of reach
      // sends an 'M' to tell the rover to 
      // move closer
    {
      testCoor.write("M");
      Serial.println("M");
      homeflag = 1;
    }
    else if (cur_state == 3)   // asks rover if the rock is the correct rock
    {
      testCoor.write("R");
      Serial.println("R");
    }
    nextHeartBeatms = millis() + HrtBtInterval;           
  } 
}

void homepos()
{
  sRotNew = HsROT;
  sFlexNew = HsFLEX;
  eFlexNew = HeFLEX;
  wrFlexNew = HwrFLEX;
  wrRotNew = HwrROT;
  gripNew = HGRIP;
  cur_state = 1;
  updatesRot();
  updatesFlex();
  updateeFlex();
  updatewrFlex();
  updatewrRot();
  updategrip();
  printServos();
  cur_state = 72;
  homeflag = 0;
  coordinates = 0;
  getrock = 0;
  saverock = 0;
  wrongrock = 0;
}

// Used to zero in on the (x,y) coordinate for a constant update
void xyZero()
{
  readCoord();
  cur_state = 67;
  while (x < xZeroed - xyERR || x > xZeroed + xyERR)
  {
    if(x < xZeroed - xyERR)
    {
      //x = x + 5;
      sRotNew = sRot - 1;
      updatesRot();
      //Serial.println("x < 155");
      readCoord();
      Serial.print("x = ");
      Serial.println(x);
      Serial.print("\tsRot = ");
      Serial.println(sRot);
    }
    if(x > xZeroed + xyERR)
    {
      //x = x - 5;
      sRotNew = sRot + 1;
      updatesRot();
      //Serial.println("x > 165");
      readCoord();
      Serial.print("x = ");
      Serial.print(x);
      Serial.print("\tsRot = ");
      Serial.println(sRot);
    }
  }
  while (y < yZeroed - xyERR || y > yZeroed + xyERR)
  {
    if(y < yZeroed - xyERR)
    {
      //y = y + 5;
      wrFlexNew = wrFlex + 1;
      updatewrFlex();
      //Serial.println("y < 95");
      readCoord();
      Serial.print("y = ");
      Serial.print(y);
      Serial.print("\twrFlex = ");
      Serial.println(wrFlex);
    }
    if(y > yZeroed + xyERR)
    {
      //y = y - 5;
      wrFlexNew = wrFlex - 1;
      updatewrFlex();
      Serial.println("y > 105");
      readCoord();
      Serial.print("y = ");
      Serial.println(y);
      Serial.print("\twrFlex = ");
      Serial.println(wrFlex);
    }
  }
  //sRotNew = 108;
  //wrFlexNew = 90;
  //updatesRot();
  //updatewrFlex();
  printServos();
  ignoreCoords = true;
  coordinates = 0;
  getrock = 1;
}

// Used to find new angles to pick up the rock
// the goal of this function is to reassign angles for each joint
// such that the gripper is hovering directly over the rock
void pickup()
{
  Gripper.write(0);  // open the gripper
  gripNew = 0;
  grip = gripNew;
  theta = 180 - HeFLEX + wrFlex - 90;  // angle between ground and gripper opening

  a = HT * tan(theta*PI/180);    // the horizontal distance from wrist flex joint to rock

  f = FA * sin((180-HeFLEX)*PI/180);   // the horizontal distance from shoulder joint to wrist

  h = sqrt(UA*UA + FA*FA - 2*UA*FA*cos((180-HeFLEX)*PI/180));  // hypotenuse of the shoulder-wrist triangle, used to find g

  g = sqrt(h*h -f*f);     // the vert distance from shoulder to wrist joint

  D = sqrt((a+f+CAMOFST)*(a+f+CAMOFST) + (g+HT-GLOFST)*(g+HT-GLOFST));   // the distance the shoulder needs to be from the wrist in order to pick up rock

  Serial.print("theta = ");
  Serial.println(theta);
  Serial.print("a = ");
  Serial.println(a);
  Serial.print("f = ");
  Serial.println(f);
  Serial.print("h = ");
  Serial.println(h);
  Serial.print("D = ");
  Serial.println(D);
  Serial.print("g = ");
  Serial.println(g);

  if (D > REACH)
  {
    cur_state = 2;
    Serial.write("M");
    homeflag = 1;
  }
  else if (D <= REACH)
  {
    eFlexNew = 130;
    updateeFlex();
    wrFlexNew = 180;
    updatewrFlex();

    eFlexNew = 180 - acos((D*D - UA*UA - FA*FA)/(-2*FA*UA))*180/PI;

    sFlexNew = HsFLEX - (180 - atan((a+f)/(g + HT - GLOFST))*180/PI - asin(FA*sin(eFlexNew*PI/180)/D)*180/PI);                                                                                                                              

    wrFlexNew = 225 - (sFlexNew + 180 - eFlexNew);
    Serial.print("eFlexNew = ");
    Serial.println(eFlexNew);
    Serial.print("sFlexNew = ");
    Serial.println(sFlexNew);
    Serial.print("wrFlexNew = ");
    Serial.println(wrFlexNew);

    cur_state = 71;
    sRotNew = sRot + 5;
    wrRotNew = 180;
    wrFlexNew = wrFlexNew - 4;
    updatesRot();
    updatesFlex();
    updatewrRot();
    updateeFlex();
    updatewrFlex();

    gripNew = 100;   // close the gripper to grasp rock, arm should be in position at this point
    updategrip();
    eFlexNew = 130;
    wrFlexNew = 180;
    printServos();
    updatewrFlex();
    updateeFlex();
    homepos();
    cur_state = 3;
  }
  getrock = 0;
}


void droprock()
{
  sRotNew = 180;  // turn arm to the side
  eFlexNew = 130;  // extend elbow to drop rock away from rover
  wrFlexNew = 90; // extend wrist straight out
  gripNew = 0;  // open the gripper all the way to drop the rock
  wrRotNew = 0;  // turn gripper parallel to ground 
  cur_state = 78;
  updateeFlex();
  updatewrFlex();
  updatesRot();
  updatewrRot();
  updatesFlex();
  updategrip();
  printServos();
  homepos();
  printServos();
  wrongrock = 0;
}

void placerock()
{
  sRotNew = 90;
  sFlexNew = 140;
  eFlexNew = 0;
  wrFlexNew = 170;
  wrRotNew = 0;
  gripNew = 0;
  cur_state = 83;
  updatesRot();
  updatesFlex();
  updateeFlex();
  updatewrFlex();
  updatewrRot();
  updategrip();
  printServos();
  homepos();
  printServos();
  saverock = 0;
}

// read (x,y) coordinates in the format of 'C:160:100'
// save the first '000' as int x
// save the second '000' as int y

void readCoord()
{
  stringComplete = false;  // set to false to stay in while loop until string is complete
  //serialEvent();
  Serial.println("Inside serialEvent()");
  while (!stringComplete)
  {
    //Serial.println("stringComplete = false");
    char inChar[] = " ";  // null terminated one-byte string
    while (testCoor.available()) 
    {
      // get the new byte:
      inChar[0] = testCoor.read(); 
      // add it to the inputString:
      inputString += inChar;
      
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if (inChar[0] == '\n') 
      {
        stringComplete = true;
        coordinateString = inputString;
        inputString = "";
      }
      Serial.println(inputString);
    }
  }
  //String coordinateString = "C:234:69";
  int firstColon = coordinateString.indexOf(':');
  Serial.println("The index of : in the string " + coordinateString + " is " + firstColon);
  int secondColon = coordinateString.indexOf(':', firstColon + 1);

  String firstValue = coordinateString.substring(firstColon + 1, secondColon);
  Serial.println("firstValue = " + firstValue);
  x = firstValue.toInt();
  Serial.println(x);
  String secondValue = coordinateString.substring(secondColon + 1);
  Serial.println("secondValue = " + secondValue);
  y = secondValue.toInt();
  Serial.println(y);


  delay(2000);
}

void readMessage()
{
  if(testCoor.available() > 0)
  {
    command_state = testCoor.read();
    Serial.println(command_state);
    if (ignoreCoords && command_state != 'H' && command_state != 'G' && command_state != 'S' && command_state != 'N')
    {
      Serial.println("ignoring coordinates");
    }
    else if (command_state == 'H')  // "H" = 72 in ASCII
    {
      homeflag = 1;
      coordinates = 0;
      getrock = 0;
      saverock = 0;
      wrongrock = 0;
      ignoreCoords = false;
      cur_state = 72;
    }
    else if (command_state == 'C')  // "C" = 67 in ASCII
    {
      homeflag = 0;
      coordinates = 1;
      getrock = 0;
      saverock = 0;
      wrongrock = 0;
      ignoreCoords = false;
      cur_state = 67;
    }
    else if (command_state == 'G')  // "G" = 71 in ASCII
    {
      homeflag = 0;
      coordinates = 0;
      getrock = 1;
      saverock = 0;
      wrongrock = 0;
      ignoreCoords = false;
      cur_state = 71;
    }
    else if (command_state == 'S')  // "S" = 83 in ASCII
    {
      homeflag = 0;
      coordinates = 0;
      getrock = 0;
      saverock = 1;
      wrongrock = 0;
      ignoreCoords = false;
      cur_state = 83;
    }
    else if (command_state == 'N')  // "N" = 78 in ASCII
    {
      homeflag = 0;
      coordinates = 0;
      getrock = 0;
      saverock = 0;
      wrongrock = 1;
      ignoreCoords = false;
      cur_state = 78;
    }

  }
  /*  Serial.print(homeflag);
   Serial.print(coordinates);
   Serial.print(getrock);
   Serial.print(saverock);
   Serial.println(wrongrock);
   */}

// Used to update all servos to global variables
void updatesRot()
{
  if(sRotNew > sRot)  //rotate base to the right
  {
    for(sRot = sRot; sRot < sRotNew; sRot += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      ShoulderRot.write(sRot);             // tell servo to go to position in variable 'pos' 
      beacon();
      delay(40);    // waits 15ms for the servo to reach the position 
    } 
  }
  else if(sRotNew < sRot) //rotate base to the left
  {
    for(sRot = sRot; sRot > sRotNew; sRot-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      beacon();
      ShoulderRot.write(sRot);              // tell servo to go to position in variable 'pos' 
      delay(40);            // waits 15ms for the servo to reach the position 
    } 
  }
  else if(sRot == sRotNew)
  {
  } 
}

void updatesFlex()
{
  if(sFlexNew > sFlex)  //rotate base to the right
  {
    for(sFlex = sFlex; sFlex < sFlexNew; sFlex += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      beacon();
      ShoulderFlex.write(sFlex);              // tell servo to go to position in variable 'pos' 
      delay(30);    // waits 15ms for the servo to reach the position 
    } 
  }
  else if(sFlexNew < sFlex) //rotate base to the left
  {
    for(sFlex = sFlex; sFlex > sFlexNew; sFlex-=1)     // goes from 180 degrees to 0 degrees 
    {
      beacon();
      ShoulderFlex.write(sFlex);              // tell servo to go to position in variable 'pos' 
      delay(30);            // waits 15ms for the servo to reach the position 
    } 
  }
  else if(sFlex == sFlexNew)
  {
  }
}

void updateeFlex()
{

  if(eFlexNew > eFlex)  //rotate base to the right
  {
    for(eFlex = eFlex; eFlex < eFlexNew; eFlex += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      ElbowFlex.write(eFlex);              // tell servo to go to position in variable 'pos' 
      beacon();
      delay(30);    // waits 15ms for the servo to reach the position 
    } 
  }
  else if(eFlexNew < eFlex) //rotate base to the left
  {
    for(eFlex = eFlex; eFlex > eFlexNew; eFlex-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      ElbowFlex.write(eFlex);              // tell servo to go to position in variable 'pos' 
      beacon();
      delay(30);            // waits 15ms for the servo to reach the position 
    } 
  }
  else if(eFlex == eFlexNew)
  {
  }
}

void updatewrFlex()
{
  if(wrFlexNew > wrFlex)  //rotate base to the right
  {
    for(wrFlex = wrFlex; wrFlex < wrFlexNew; wrFlex += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      WristFlex.write(wrFlex);              // tell servo to go to position in variable 'pos' 
      beacon();
      delay(40);    // waits 15ms for the servo to reach the position 
    } 
  }
  else if(wrFlexNew < wrFlex) //rotate base to the left
  {
    for(wrFlex = wrFlex; wrFlex > wrFlexNew; wrFlex-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      WristFlex.write(wrFlex);              // tell servo to go to position in variable 'pos' 
      beacon();
      delay(40);            // waits 15ms for the servo to reach the position 
    } 
  }
  else if(wrFlex == wrFlexNew)
  {
  }
}

void updatewrRot()
{
  if(wrRotNew > wrRot)  //rotate base to the right
  {
    for(wrRot = wrRot; wrRot < wrRotNew; wrRot += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      WristRot.write(wrRot);              // tell servo to go to position in variable 'pos' 
      beacon();
      delay(15);    // waits 15ms for the servo to reach the position 
    } 
  }
  else if(wrRotNew < wrRot) //rotate base to the left
  {
    for(wrRot = wrRot; wrRot > wrRotNew; wrRot-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      WristRot.write(wrRot);              // tell servo to go to position in variable 'pos' 
      beacon();
      delay(15);            // waits 15ms for the servo to reach the position 
    } 
  }
  else if(wrRot == wrRotNew)
  {
  }
}

void updategrip()
{
  if(gripNew > grip)  //rotate base to the right
  {
    for(grip = grip; grip < gripNew; grip += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      Gripper.write(grip);              // tell servo to go to position in variable 'pos' 
      beacon();
      delay(15);    // waits 15ms for the servo to reach the position 
    } 
  }
  else if(gripNew < grip) //rotate base to the left
  {
    for(grip = grip; grip > gripNew; grip-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      Gripper.write(grip);              // tell servo to go to position in variable 'pos' 
      beacon();
      delay(15);            // waits 15ms for the servo to reach the position 
    } 
  }
  else if(grip == gripNew)
  {
  }
}

void printServos()
{
  Serial.println("Servos Updated");
  Serial.print("sRot = ");
  Serial.println(sRot);
  Serial.print("sFlex = ");
  Serial.println(sFlex);
  Serial.print("eFlex = ");
  Serial.println(eFlex);
  Serial.print("wrFlex = ");
  Serial.println(wrFlex);
  Serial.print("wrRot = ");
  Serial.println(wrRot);
  Serial.print("grip = ");
  Serial.println(grip);  
}

