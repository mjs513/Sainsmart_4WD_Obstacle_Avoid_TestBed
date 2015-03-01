
//Hardware set the pins
// L298D Motor Controller
#define ENA 5  //enable A on pin 5 (must be a pwm pin)   Speed Control
#define ENB 3  //enable B on pin 3 (must be a pwm pin)   Speed Control

#define IN1 7  //IN1 on pin controls Motor A  Motor A Right Motor (view from front)
#define IN2 6  //IN2 on pin controls Motor A
#define IN3 4  //IN3 on pin conrtols Motor B  Motor B Left Motor (view from front)
#define IN4 2  //IN4 on pin controls Motor B 

const int rightmotorpin1 = IN1;  //signal output 1 of Dc motor 
const int rightmotorpin2 = IN2;  //signal output 2 of Dc motor 
const int leftmotorpin1  = IN3;  //signal output 3 of Dc motor 
const int leftmotorpin2  = IN4;  //signal output 4 of Dc motor 


//set interrupts and encoder pins
const int l_int = 5;
const int r_int = 4;
const int l_encoder = 18;
const int r_encoder = 19;

const int HeadServopin = 8; // signal input of headservo
//const int head_tilt_pin = 3; // signal input for headservo tilt

NewPing sonarll(27, 26, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarlc(32, 33, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarlr(28, 29, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarhd(49, 48, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
