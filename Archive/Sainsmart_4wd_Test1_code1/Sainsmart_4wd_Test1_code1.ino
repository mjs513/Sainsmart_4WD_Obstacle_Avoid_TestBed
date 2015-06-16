#include <Servo.h>

const int HeadServopin = 4; // signal input of headservo

//Pins for motor A 
const int MotorA1 = 8;
const int MotorA2 = 9;

//Pins for motor B 
const int MotorB1 = 6;
const int MotorB2 = 7;

//Pins for ultrasonic sensor
const int trigger = 27;
const int echo = 26;

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
char choice;
 
char contcommand;
int modecontrol=0;
int power=1;

const int distancelimit = 27; //Distance limit for obstacles in front           
const int sidedistancelimit = 12; //Minimum distance in cm to obstacles at both sides (the robot will allow a shorter distance sideways)

int distance;
int numcycles = 0;
char turndirection; //Gets 'l', 'r' or 'f' depending on which direction is obstacle free
const int turntime = 900; //Time the robot spends turning (miliseconds)
int thereis;
Servo head;

void setup(){
  head.attach(HeadServopin);
  head.write(80);

  pinMode(MotorA1, OUTPUT); 
  pinMode(MotorA2, OUTPUT); 
  pinMode(MotorB1, OUTPUT); 
  pinMode(MotorB2, OUTPUT);
  
  pinMode(trigger,OUTPUT);
  pinMode(echo,INPUT);
  
  //Variable inicialization
  digitalWrite(MotorA1,LOW);
  digitalWrite(MotorA2,LOW);
  digitalWrite(MotorB1,LOW);
  digitalWrite(MotorB2,LOW);
  
  digitalWrite(trigger,LOW);
}

void go(){ 
   digitalWrite (MotorA1, HIGH);                              
   digitalWrite (MotorA2, LOW); 
   digitalWrite (MotorB1, HIGH); 
   digitalWrite (MotorB2, LOW);
}

void backwards(){
  digitalWrite (MotorA1 , LOW);                              
  digitalWrite (MotorA2, HIGH); 
  digitalWrite (MotorB1, LOW); 
  digitalWrite (MotorB2, HIGH);
}

int watch(){
  long howfar;
  
  digitalWrite(trigger,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(trigger,HIGH);
  delayMicroseconds(15);
  digitalWrite(trigger,LOW);
  
  howfar=pulseIn(echo,HIGH);
  howfar=howfar*0.01657; //how far away is the object in cm
  return round(howfar);
}

void turnleft(int t){
  digitalWrite (MotorA1, LOW);                              
  digitalWrite (MotorA2, HIGH); 
  digitalWrite (MotorB1, HIGH); 
  digitalWrite (MotorB2, LOW);
  delay(t);
}

void turnright(int t){
  digitalWrite (MotorA1, HIGH);                              
  digitalWrite (MotorA2, LOW); 
  digitalWrite (MotorB1, LOW); 
  digitalWrite (MotorB2, HIGH);
  delay(t);
}  

void stopmove(){
  digitalWrite (MotorA1 ,LOW);                              
  digitalWrite (MotorA2, LOW); 
  digitalWrite (MotorB1, LOW); 
  digitalWrite (MotorB2, LOW);
}  

void watchsurrounding(){ //Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval, 
                         //leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)

  centerscanval = watch();
  if(centerscanval<distancelimit){stopmove();}
  
  head.write(120);
  delay(100);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){stopmove();}

  head.write(160); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){stopmove();}
  
  head.write(120);
  delay(100);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){stopmove();}

  head.write(80); //I used 80 degrees because its the central angle of my 160 degrees span (use 90 degrees if you are moving your servo through the whole 180 degrees)
  delay(100);
  centerscanval = watch();
  if(centerscanval<distancelimit){stopmove();}

  head.write(40);
  delay(100);
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){stopmove();}

  head.write(0);
  delay(100);
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){stopmove();}

  head.write(80); //Finish looking around (look forward again)
  delay(300);
}

char decide(){
  watchsurrounding();
  if (leftscanval>rightscanval && leftscanval>centerscanval){
    choice = 'l';
  }
  else if (rightscanval>leftscanval && rightscanval>centerscanval){
    choice = 'r';
  }
  else{
    choice = 'f';
  }
  return choice;
}

void loop(){
  if(power==1){
    go();  // if nothing is wrong go forward using go() function above.
    ++numcycles;
    if(numcycles>130){ //Watch if something is around every 130 loops while moving forward 
      watchsurrounding();
      if(leftscanval<sidedistancelimit || ldiagonalscanval<distancelimit){
        turnright(turntime);
      }
      if(rightscanval<sidedistancelimit || rdiagonalscanval<distancelimit){
        turnleft(turntime);
      }
      numcycles=0; //Restart count of cycles
    }
    distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
    if (distance<distancelimit){ // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
      ++thereis;}
    if (distance>distancelimit){
        thereis=0;} //Count is restarted
    if (thereis > 25){
      stopmove(); // Since something is ahead, stop moving.
      turndirection = decide(); //Decide which direction to turn.
      switch (turndirection){
        case 'l':
          turnleft(turntime);
          break;
        case 'r':
          turnright(turntime);
          break;
        case 'f':
          ; //Do not turn if there was actually nothing ahead
          break;
      }
      thereis=0;
    }
   }
}

