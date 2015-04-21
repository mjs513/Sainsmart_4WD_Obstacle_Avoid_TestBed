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

void decide_direction() {
  if(obs_array[3] == 1 && obs_array[0] == 0 && obs_array[1] == 0 && obs_array[2] == 0) {
       telem.println("OBSTACLE HEIGHT - STOP or REVERSE");
       decide_direction_head(); 
       return;
     }
  if(obs_array[0] == 1 && obs_array[2] == 1) {
       telem.println("STOP or Reverse ");
       brake();
       decide_direction_head(); 
       return;
   } 
   
  if(obs_array[3] == 0 && obs_array[0] == 0 && obs_array[1] == 0 && obs_array[2] == 0) {
      telem.println("NO OBSTACLES");
      //moveForward(motorSpeed);
      moveForward();
      //currentTime = millis();
      //while(!IsTime(&currentTime, interval)){
      //   encoder_l();
      //   encoder_r();
      //}
      
      while(obs_array[3] != 1 || obs_array[0] != 1 || obs_array[1] != 1 || obs_array[2] != 1) {
        //Cycle through obstacle avoidance sensors for emergency stop
        read_sensors();
        oneSensorCycle();
        if(obs_array[3] == 1 || obs_array[0] == 1 || obs_array[1] == 1 || obs_array[2] == 1) {
          brake();
          return;
        }
      }
   }
   
  if(obs_array[0] == 1  && obs_array[1] == 0  && obs_array[2] == 0) {
       telem.println("33% Left Blocked");
       brake();
       body_rturn();
       delay(400);	  //was 1000 
       brake();
       return;
   }
  if(obs_array[0] == 0  && obs_array[1] == 0  && obs_array[2] == 1) {
       telem.println("33% Right Blocked");
       brake();
       body_lturn();
       delay(400);	//was 1000	   
       brake();
       return;
   }
  if(obs_array[0] == 1  && obs_array[1] == 1  && obs_array[2] == 0) {
       telem.println("50% Left Blocked");
       brake(); 
       body_rturn();
       delay(700);     //was 1500	   
       brake();
       return;
   }
  if(obs_array[0] == 0  && obs_array[1] == 1  && obs_array[2] == 1) {
       telem.println("50% Right Blocked");
       brake(); 
       body_lturn();
       delay(700);	 //was 1500	   
       brake();
       return;
   }  
}


