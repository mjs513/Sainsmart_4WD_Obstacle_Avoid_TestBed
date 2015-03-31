// ---------------------------------------------------------------------------
// Code based on example code from following:
// Sainsmart Obstacle Avoidance Robot:
//    http://www.mkme.org/index.php/arduino-sainsmart-4wd-robot/
//    https://github.com/gmossy/Sainsmart-4WD-Robot
// Wheel Encoders - the DAGU Simple Encoder Kit
//    http://www.bajdi.com/adding-encoders-to-those-cheap-yellow-motors/
//    http://letsmakerobots.com/node/38636
//    http://playground.arduino.cc/Main/ReadingRPM
//    http://elimelecsarduinoprojects.blogspot.com/2013/06/measure-rpms-arduino.html 
//  Compass Averaging
//    Yamartino Library, Christopher Baker  https://github.com/SAIC-ATS/Algorithms.git
//  Obstacle avoidance approaches
//    http://homepages.engineering.auckland.ac.nz/~pxu012/mechatronics2013/group7/index.html
//    http://homepages.engineering.auckland.ac.nz/~pxu012/mechatronics2013/group7/software.html
//  IR Sensing
//    http://letsmakerobots.com/node/40502
//    http://adhocnode.com/arduino-irrecv-module/
//    http://arduino-info.wikispaces.com/IR-RemoteControl
//
//  More to follow as I add PID controls and Bubble rebound algorithm for obstacle
//  avoidance
// -------------------------------------------------------------------


#include <NewPing.h>
#include <Servo.h>         //servo library
#include <Wire.h>
#include <LSM303.h>
#include <Yamartino.h>

#include "Constants.h"
#include "IOpins.h"

Yamartino yamartino(40);
LSM303 compass;

Servo headservo;

// the interval in mS 
unsigned long currentTime;
unsigned long currentTime_avg;

unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

uint8_t obs_array[5];
//const int obsDist = 27;
const int obsDist = 47;
const int sidedistancelimit = 45;
unsigned int cm_head[5];

float heading;

int roam = 0;
//int motorSpeed_right = 87;      //define motor speed parameter which will be mapped as a percentage value
//int motorSpeed_left = 75;      // these are reversed right is left when looking from behind
int motorSpeed_right = 75;     //define motor speed parameter which will be mapped as a percentage value
int motorSpeed_left = 87;      // 

int turnSpeed = 75;             //define turning speed parameter

int Speed;

// read RPM
 volatile byte half_revolutions_l;
 unsigned int rpm_l;
 unsigned long timeold_l;
 unsigned int rpm_l_avg;
 unsigned int rpm_l_index;
 
 volatile byte half_revolutions_r;
 unsigned int rpm_r;
 unsigned int rpm_r_avg;
 unsigned int rpm_r_index;
 unsigned long timeold_r;
 
void setup() {
    telem.begin(9600);
    
    telem.println("Ready to receive telem Commands![f, b, r, l, s, t]"); // Tell us I"m ready
    //telem.println("My Commands are: ");
    //telem.println("c:carpet");
    //telem.println("f:forward");
    //telem.println("b:backward");
    //telem.println("r:right");
    //telem.println("l:left");
    //telem.println("s:stop");
    //telem.println("t:toggleRoam");    
    
    //compass.init();
    //compass.enableDefault();
  
    /*
    Calibration values; the default values of +/-32767 for each axis
    lead to an assumed magnetometer bias of 0. Use the Calibrate example
    program to determine appropriate values for your particular unit.
    */
    //compass.m_min = (LSM303::vector<int16_t>){-3061,  -2357,  -2424};
    //compass.m_max = (LSM303::vector<int16_t>){+2839,  +3278,  +3135}; 
	
    //signal output port
    //set all of the outputs for the motor driver
    pinMode(IN1, OUTPUT);       // Motor Driver 
    pinMode(IN2, OUTPUT);       // Motor Driver
    pinMode(IN3, OUTPUT);       // Motor Driver
    pinMode(IN4, OUTPUT);       // Motor Driver
 
    //Enable pull ups on encoders
    digitalWrite(l_encoder, HIGH);
    digitalWrite(r_encoder, HIGH);
    //Attach interrupts for encoders
    attachInterrupt(l_int, rpm_fun_l, CHANGE);   
    half_revolutions_l = 0;
    rpm_l = 0;
    timeold_l = 0;
    attachInterrupt(r_int, rpm_fun_r, CHANGE);   
    half_revolutions_r = 0;
    rpm_r = 0;
    timeold_r = 0;
  
    // headservo interface
    headservo.attach(HeadServopin);
    headservo.write(head_fwd);
    delay(100);
}

void loop() {
    if (telem.available() > 0)
    {
    int val = telem.read();	//read telem input commands

    switch(val)
    {
    case 'c' :
      turnSpeed = 85;
      break;
      
    case 'f' : 
      telem.println("Rolling Forward!");
      moveForward();
      rpm_r_index = 0;  rpm_l_index = 0;
      rpm_r_avg = 0;    rpm_l_avg = 0;
      currentTime = millis();
      while(!IsTime(&currentTime, interval)){
        //Read encoders and calculate RPM
         encoder_l();   encoder_r();
         rpm_r_index++; rpm_l_index++;
         rpm_r_avg = rpm_r_avg + rpm_r; 
         rpm_l_avg = rpm_l_avg + rpm_l;
         telem.println(rpm_l_avg/rpm_l_index); telem.println(rpm_r_avg/rpm_r_index);
         
         //Cycle through obstacle avoidance sensors for emergency stop
         read_sensors();
         oneSensorCycle();
         if(obs_array[3] == 1 || obs_array[0] == 1 || obs_array[1] == 1 || obs_array[2] == 1) {
           //Serial.println(rpm_l_avg/rpm_l_index); Serial.println(rpm_r_avg/rpm_r_index); 
           brake();
           return; 
         }
      }
      telem.println(rpm_l_avg/rpm_l_index); telem.println(rpm_r_avg/rpm_r_index); 
      brake();
      delay(1000);
      break;
      
    case 'l' : 
      telem.println("Turning Left!");
      body_lturn(turnSpeed);
      delay(400);  //was 2000
      brake();
      delay(1000);  //was 5000
      break;
      
    case 'r' :   
      telem.println("Turning Right!");
      body_rturn(turnSpeed);
      delay(400);
      brake();
      delay(1000);  //was 5000
      break;       
   case 'b' :    
      telem.println("Moving Backward!");
      //moveBackward(motorSpeed);
      moveBackward();
      delay(700);
      brake();
      delay(1000);  //was 5000
      break;
   case 's' :      
      telem.println("Stop!");
      brake();
      delay(1000);  //was 5000
      break;
   case 't' :      
      telem.println("toggle Roam Mode"); 
      toggleRoam();
      break;
    }      
    delay(1);  
    telem.println("I'm Ready to receive telem Commands![f, b, r, l, s, t]"); // Tell us I"m ready
  }
             
  if(roam == 0){ 
      //just listen for telem commands and wait
      }
  else if(roam == 1){  //If roam active- drive autonomously
    goRoam();
    }  
}

void toggleRoam(){
  // This method chooses to make the robot roam or else use the telem command input.
  if(roam == 0){
   roam = 1;
   telem.println("Activated Roam Mode");
  } else {
    roam = 0;
    brake();
    telem.println("De-activated Roam Mode");
  }
}

void goRoam() {  
    read_sensors();
    // The rest of your code would go here.
    oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
    
    decide_direction();
}




