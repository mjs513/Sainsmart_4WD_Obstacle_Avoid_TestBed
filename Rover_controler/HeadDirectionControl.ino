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

unsigned int pos1 = 0;    // variable to store the servo position
int delta_t, blocked;
float numer, denom;
unsigned int sonar_dist[9];
float bubble_boundary;

void decide_direction_head() {
  brake();
  // establish variables for duration of the ping, 
  // and the distance result in inches and centimeters:
  int start_time = millis();
  
  for(int i =0; i < N; i++)     // goes from 180 degrees to 0 degrees 
  {
    headservo.write(i*angle);               // tell servo to go to position in variable 'pos' 
    delay(550);                             // waits 15ms for the servo to reach the position 
   
    unsigned int uS = sonarhd.ping(); 
    sonar_dist[i] = uS / US_ROUNDTRIP_CM;
    if(sonar_dist[i] == 0) sonar_dist[i] = 999;
    telem << ( sonar_dist[i]) << ", ";
    delay(60); 
  } 
  telem << endl;
  
  float alpha0 = PI/N;
  numer = 0;  denom =0;
  for(int i=-N/2; i <= N/2; i++) {
      numer = numer + radians(alpha0*i)*sonar_dist[i];
      denom = denom + sonar_dist[i];
  }
  
  float rebound_angle = degrees(numer/denom);
  telem << "REBOUND ANGLE = " << rebound_angle << " degrees" << endl;  

  compass_update(); 
  telem << "Current heading: " << yar_heading << endl; 
  
  float new_heading = yar_heading + rebound_angle;
  telem << "Actual calc for new heading: " << new_heading << endl;
  
  if(new_heading > 360.0f)
     new_heading -= 360.0f;
   else if(new_heading < 0.0f)
     new_heading = 360.0f - new_heading;
  
  boolean clockwise;
  clockwise = true;
  
  float angle_delta = abs(new_heading - yar_heading);
  clockwise = (rebound_angle >= 0.0f);
  
  telem << "New Heading: " << new_heading << "  Clockwise: " << clockwise << endl;
  telem << "Angle delta: " << angle_delta << endl;
  telem << endl;
  /*
  while(angle_delta > 0.5f){
    if(clockwise)
      body_rturn(65);
     else      
      body_lturn(65);
    compass_update();
    angle_delta = abs(new_heading - yar_heading);
  }
  brake();
  */
  headservo.write(head_fwd);

  head_distance();
  
  if(obs_array[0] == 1 && obs_array[1] == 1 && obs_array[2] == 1 && obs_array[3] == 1 && obs_array[4] == 1) {
       telem.println("All Directions Blocked - REVERSE");
       //moveBackward(motorSpeed);
       moveBackward();
       delay(500);	   
       return;
   }
   
  if(cm_head[0] > cm_head[4] &&  cm_head[0] > cm_head[2] ) {
       telem.println("Head - turn left ");
       body_lturn();
       delay(700);	 	   
       return;
   }       

  if(cm_head[4] > cm_head[0] &&  cm_head[4] > cm_head[0] ) {
       telem.println("Head - turn right");
       body_rturn();
       delay(700);	 	   
       return;
   }

  if(cm_head[1] > cm_head[3] &&  cm_head[1] > cm_head[2] ) {
       telem.println("Head - turn left ");
       body_lturn();
       delay(700);	 	   
       return;
   }       

  if(cm_head[3] > cm_head[1] &&  cm_head[3] > cm_head[0] ) {
       telem.println("Head - turn right");
       body_rturn();
       delay(700);	 	   
       return;
   }
  
  if (cm_head[4] < sidedistancelimit && cm_head[0] < sidedistancelimit && cm_head[2] < obsDist) {
      //moveBackward(motorSpeed);
      moveBackward();
      delay(500);
      return;
  } 

}

