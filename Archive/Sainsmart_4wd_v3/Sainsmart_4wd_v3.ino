/* 
Following code by EricWillaim
All build information, code and forum posts/discussion can be found at:
http://mkme.org
Dedicated build page:
http://www.mkme.org/index.php/arduino-sainsmart-4wd-robot/
This is my attempt at programming the Sainsmart 4WD robot
So far it now checks for a new direction periodically and performs
backup and turn functions as in reaction to the environment it is presented.
It is by far not optimized code- but works for now :)
I will update this periodically as I get time to learn new methods and improve the code. 
Please check out my Youtube videos here and consider a thumbs up if this helped you!
Youtube : http://www.youtube.com/user/Shadow5549
Some portions of this code adapted from:
http://www.duino-robotics.com/
V9 Dec 12 2013:
Added a forward distance check after turns- will continue to turn if distance still too small
Minor Tweaks but still using delay functions
V10 May 2014
Added support for Roam mode- serial commands will toggle roam versus individual driving commands
*/

#include <Servo.h> //servo library
//#include <NewPing.h> //Ping Library

//sets up mode filter
#include <ModeFilter.h> 
ModeFilter mfilter;
byte i = 0;
int modE;

Servo headservo, head_tilt_servo;
const int numReadings = 12;   // set a variable for the number of readings to take
int index = 0;                // the index of the current reading

//sets up headservo sensor Echo and Trig pins
const int EchoPin = 48; // Ultrasonic signal input
const int TrigPin = 49; // Ultrasonic signal output

//lower left sensor
#define TRIGGER_PIN  27  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     26  // Arduino pin tied to echo pin on the ultrasonic sensor.

//lower rigth sensro
#define echoPin 29 // Echo Pin
#define trigPin 28 // Trigger Pin

//front middle
#define echoPin 33 // Echo Pin
#define trigPin 32 // Trigger Pin

//sets up servo angles
const int head_fwd = 90; // center is 80 until i fix
const int head_lft = 0;
const int head_rt = 180;
const int head_ldiag = 45;
const int head_rdiag = 135;
const int head_down = 130;
const int head_up = 90;

//Scan limints and variables for head servo sensor
int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
int dleftscanval, dcenterscanval, drightscanval, dldiagonalscanval, drdiagonalscanval;
const int distancelimit = 35; //Distance limit for obstacles in front, was 27           
const int sidedistancelimit = 25; //Minimum distance in cm to obstacles at both sides (the robot will allow a shorter distance sideways)

//Motor control
const int leftmotorpin1 = 8; //signal output of Dc motor driven plate
const int leftmotorpin2 = 9;
const int rightmotorpin1 = 6;
const int rightmotorpin2 = 7;
const int motor_speed = 90;

//#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
//NewPing sonar(TrigPin, EchoPin, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
int maximumRange = 200; // Maximum range needed
int minimumRange = 0; // Minimum range needed
long duration, distance; // Duration used to calculate distance

const int HeadServopin = 4; // signal input of headservo
const int head_tilt_pin = 3; // signal input for headservo tilt

const int maxStart = 800; //run dec time- no clue what this is
unsigned long time; //used for upcoming improvements (time used instead of loops)
unsigned long time1; //used for upcoming improvements (time used instead of loops)
int add= 0; //used for nodanger loop count
int add1= 0;  //used for nodanger loop count
int roam = 1;
int currDist = 5000; // distance
int currDistDwn = 5000; // distance
boolean running = false;// This is from old code examples


void setup() {
  
  Serial.begin(9600); // Enables Serial monitor for debugging purposes
  Serial.println("Serial Data Initiated!"); // Test the Serial communication
  
  //signal output port
  for (int pinindex = 5; pinindex < 10; pinindex++) {
    pinMode(pinindex, OUTPUT); // set pins 5 to 10 as outputs
  }
  
  //set head servo signal path
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  
  // headservo interface
  headservo.attach(HeadServopin);
  head_tilt_servo.attach(head_tilt_pin);
  
  //start buffer movable head
  headservo.write(head_lft);
  delay(500);
  headservo.write(head_ldiag);
  delay(500);
  headservo.write(head_rt);
  delay(500);
  headservo.write(head_rdiag);
  delay(500);
  headservo.write(head_fwd);
  delay(500);

  
}

void loop() {
  if (Serial.available() > 0) {
    int data = Serial.read();	//read serial input commands
    switch(data) {
      case 'w' : 
        Serial.println("Rolling!");
        moveForward(); 
        break;
      case 'x' : 
        backup(); 
        break;
      case 'a' : 
        body_lturn();  
        break;
      case 'd' : 
        body_rturn();  
        break;
      case 's' : 
        totalhalt(); 
        break;
      case 'r' : 
        toggleRoam(); 
        break;  
      }
  }
  
  if(roam == 0){ //just listen to serial commands and wait
   // Do something else if you like
  }
  else if(roam == 1){  //If roam active- drive autonomously  
    //time = millis(); // Sets "time" to current system time count
    headservo.write(head_fwd);  
    currDist = MeasuringDistance(); //measure front distance
      
    //head_tilt_servo.write(head_down); //Finish looking around (look forward again)
    //delay(50);
    //currDistDwn = MeasuringDistance(); //measure front distance     
    
    //head_tilt_servo.write(head_up); //Finish looking around (look forward again) 
    
    Serial.print("Current Forward Distance: ");Serial.println(currDist);
    //currDist = min(currDist, currDistDwn);
    
    if(currDist > distancelimit) {
      add = (add1++);// Start adding up the loop count done in nodanger
      nodanger();
      Serial.println("Nodanger: ");
      Serial.println(currDist);
    }
    else if(currDist < distancelimit){
      //add=0;
      Serial.println("Forward Blocked- Decide Which Way");
      Serial.println(currDist);
      backup();
      decide();
      headservo.write(head_fwd);
      //head_tilt_servo.write(head_up);
      delay(10);      
    }
 } 
}
  
//measure distance, unit “cm”
long MeasuringDistance() {
  //unsigned int uS = sonar.ping_median(numReadings); 
  //return uS / US_ROUNDTRIP_CM;
  for(i = 0; i < 6; i++){
    delay(50);
    /* The following trigPin/echoPin cycle is used to determine the
    distance of the nearest object by bouncing soundwaves off of it. */ 
    digitalWrite(TrigPin, LOW); 
    delayMicroseconds(2); 
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10); 
    digitalWrite(TrigPin, LOW);
    duration = pulseIn(EchoPin, HIGH);
    modE = mfilter.get_filtered_with_sample(duration);

  }
  
  //Calculate the distance (in cm) based on the speed of sound.
  distance = modE/58.2;
  Serial.print("Mode Distance ------------  "); Serial.println(distance);
  return distance;
}
  
// forward
void moveForward() {
   analogWrite(leftmotorpin1, 0);//Changed these to analog write for slower, was 120
   analogWrite(leftmotorpin2, motor_speed);
   analogWrite(rightmotorpin1, 0);
   analogWrite(rightmotorpin2, motor_speed);
}

void toggleRoam(){
if(roam == 0){
   roam = 1;
   Serial.println("Activated Roam Mode");
  }
  else{
    roam = 0;
    totalhalt();
    Serial.println("De-activated Roam Mode");
  }
}

void nodanger() {
  running = true;// Do I need these?
  analogWrite(leftmotorpin1, 0);//Changed these to analog write for slower
  analogWrite(leftmotorpin2, motor_speed);
  analogWrite(rightmotorpin1, 0);
  analogWrite(rightmotorpin2, motor_speed);
  if (add1 > 15 ) decide(); // Robot will stop and check direction every X loops through nodanger then resets in totalhalt (40 is good)
                            // was 38
  add1 = 0;
  return;
}

//backward
void backup() {
  add1=0; // resets the counter for the nodanger loops
  running = true;//Do I need these?
  //digitalWrite(leftmotorpin1, HIGH);
  //digitalWrite(leftmotorpin2, LOW);
  //digitalWrite(rightmotorpin1, HIGH);
  //digitalWrite(rightmotorpin2, LOW);  
  Serial.println("Backing Up");
  analogWrite(leftmotorpin1, 120);//Changed these to analog write for slower
  analogWrite(leftmotorpin2, 0);
  analogWrite(rightmotorpin1, 120);
  analogWrite(rightmotorpin2, 0);
  
  totalhalt();
  
}

//choose which way to turn
void whichway(){ //Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval, 
                 //leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
  running = false;
  
  headservo.write(head_lft);
  centerscanval = MeasuringDistance();
  if(centerscanval < distancelimit) { totalhalt(); }
  
  headservo.write(head_ldiag);
  ldiagonalscanval = MeasuringDistance();
  if(ldiagonalscanval < distancelimit) { totalhalt(); }
  
  headservo.write(head_lft); //Didn't use 180 degrees because my servo is not able to take this angle
  leftscanval = MeasuringDistance();
  if(leftscanval < sidedistancelimit) { totalhalt(); }

  headservo.write(head_ldiag);
  ldiagonalscanval = MeasuringDistance();
  if(ldiagonalscanval < distancelimit) { totalhalt(); }
  
  headservo.write(head_fwd); //I used 80 degrees because its the central angle of my 160 degrees span (use 90 degrees if you are moving your servo through the whole 180 degrees)
  centerscanval = MeasuringDistance();
  if(centerscanval < distancelimit) { totalhalt(); }

  headservo.write(head_rdiag);
  rdiagonalscanval = MeasuringDistance();
  if(rdiagonalscanval < distancelimit) { totalhalt(); }
  
  headservo.write(head_rt);
  rightscanval = MeasuringDistance();
  if(rightscanval < sidedistancelimit) { totalhalt(); }

  headservo.write(head_fwd); //Finish looking around (look forward again)
  delay(50);
  
 //======================================
 /* head_tilt_servo.write(head_down); //Finish looking around (look forward again)
  delay(50);
  
  headservo.write(head_lft);
  centerscanval = MeasuringDistance();
  if(centerscanval < distancelimit) { totalhalt(); }
  
  headservo.write(head_ldiag);
  delay(50);
  ldiagonalscanval = MeasuringDistance();
  if(ldiagonalscanval < distancelimit) { totalhalt(); }
  
  headservo.write(head_lft); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftscanval = MeasuringDistance();
  if(leftscanval < sidedistancelimit) { totalhalt(); }

  headservo.write(head_ldiag);
  delay(50);
  ldiagonalscanval = MeasuringDistance();
  if(ldiagonalscanval < distancelimit) { totalhalt(); }
  
  headservo.write(head_fwd); //I used 80 degrees because its the central angle of my 160 degrees span (use 90 degrees if you are moving your servo through the whole 180 degrees)
  delay(50);
  centerscanval = MeasuringDistance();
  if(centerscanval < distancelimit) { totalhalt(); }

  headservo.write(head_rdiag);
  delay(50);
  rdiagonalscanval = MeasuringDistance();
  if(rdiagonalscanval < distancelimit) { totalhalt(); }
  
  headservo.write(head_rt);
  delay(50);
  rightscanval = MeasuringDistance();
  if(rightscanval < sidedistancelimit) { totalhalt(); }

  head_tilt_servo.write(head_up); //Finish looking around (look forward again)
  delay(10);
  */
  headservo.write(head_fwd); //Finish looking around (look forward again)
  delay(10); 
}  

void decide(){
  whichway();
  if (leftscanval > rightscanval && leftscanval > centerscanval){
    body_lturn();
    currDist = MeasuringDistance();
  }
  else if (rightscanval > leftscanval && rightscanval > centerscanval){
    body_rturn();
    currDist = MeasuringDistance();
  }
  else if (rightscanval < sidedistancelimit && leftscanval < sidedistancelimit && centerscanval < distancelimit) {
    backup();
    decide();
  }
  else{
    //moveForward();
     currDist = MeasuringDistance();
  }  
}
 
void totalhalt() {
  digitalWrite(leftmotorpin1, HIGH);
  digitalWrite(leftmotorpin2, HIGH);
  digitalWrite(rightmotorpin1, HIGH);
  digitalWrite(rightmotorpin2, HIGH);
  Serial.println("Totalhalt!");
  headservo.write(head_fwd); // set servo to face forward
  //head_tilt_servo.write(head_up); // set servo to face forward  
  delay(250);
  running = false;//Do I need these?
  add1=0; // resets the counter for the nodanger loops
  return;
}

//turn left
void body_lturn() {
  running = true;
  analogWrite(leftmotorpin1, 0);
  analogWrite(leftmotorpin2, 220);
  analogWrite(rightmotorpin1, 220);
  analogWrite(rightmotorpin2, 0);
  Serial.println("Turning Left");
  delay(400);
}

//turn right
void body_rturn() {
  running = true;
  analogWrite(leftmotorpin1, 220);
  analogWrite(leftmotorpin2, 0);
  analogWrite(rightmotorpin1, 0);
  analogWrite(rightmotorpin2, 220);
  Serial.println("Turning Right");
  delay(400);
}

