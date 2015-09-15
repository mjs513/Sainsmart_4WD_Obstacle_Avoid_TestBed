//============================================================================
//    Sketch to test various technicques in robotic car design such as
//    obstacle detection and avoidance, compass as turn guide,
//    motor control, etc.
//    Copyright (C) 2015  Michael J Smorto
//    https://github.com/mjs513/Sainsmart_4WD_Obstacle_Avoid_TestBed.git
//    FreeIMU@gmail.com
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License along
//    with this program; if not, write to the Free Software Foundation, Inc.,
//    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

//Hardware set the pins
// L298D Motor Controller
/********************************************************
Following lines assume right as you view from the front, uncomment lines
following this section assumes viewing from behind as if you are driving
a car.
*********************************************************
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
*************************************************/
#define ENA 3  //enable A on pin 5 (must be a pwm pin)   Speed Control
#define ENB 5  //enable B on pin 3 (must be a pwm pin)   Speed Control

#define IN1 4  //IN1 on pin controls Motor A  Motor A Right Motor (view from front)
#define IN2 2  //IN2 on pin controls Motor A
#define IN3 7  //IN3 on pin conrtols Motor B  Motor B Left Motor (view from front)
#define IN4 6  //IN4 on pin controls Motor B 


const int rightmotorpin1 = IN3;  //signal output 1 of Dc motor 
const int rightmotorpin2 = IN4;  //signal output 2 of Dc motor 
const int leftmotorpin1  = IN1;  //signal output 3 of Dc motor 
const int leftmotorpin2  = IN2;  //signal output 4 of Dc motor 

//set interrupts and encoder pins
const int l_int = 4;
const int r_int = 5;
const int l_encoder = 19;
const int r_encoder = 18;

const int HeadServopin = 8; // signal input of headservo
//const int head_tilt_pin = 3; // signal input for headservo tilt

NewPing sonarll(27, 26, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarlc(32, 33, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarlr(28, 29, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarhd(49, 48, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


//IR Sensor Pins
const int leftIRsensor = A0;
const int rightIRsensor = A2;

