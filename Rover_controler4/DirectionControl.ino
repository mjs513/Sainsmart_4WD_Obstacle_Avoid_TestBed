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
       //decide_direction_head();
       //moveBackward();
       //delay(500);
       brake;
       Select_Direction(); 
       return;
     }
  if(obs_array[0] == 1 && obs_array[2] == 1) {
       telem.println("STOP or Reverse ");
       brake();
       //decide_direction_head(); 
       Select_Direction();
       return;
   } 
   
  if(obs_array[3] == 0 && obs_array[0] == 0 && obs_array[1] == 0 && obs_array[2] == 0) {
      telem.println("NO OBSTACLES");
      
      moveForward();
      
      while(obs_array[3] != 1 || obs_array[0] != 1 || obs_array[1] != 1 || obs_array[2] != 1) {
        //Cycle through obstacle avoidance sensors for emergency stop
        rpm_r_index = 0;  rpm_l_index = 0;
        rpm_r_avg = 0;    rpm_l_avg = 0;
        currentTime = millis();
        while(!IsTime(&currentTime, interval)){
        //Read encoders and calculate RPM
         encoder_l();
         encoder_r();
         if(rpm_r !=0) {
           rpm_r_index++; 
           rpm_r_avg = rpm_r_avg + rpm_r;           
         }
         if(rpm_l != 0) { 
           rpm_l_index++;
           rpm_l_avg = rpm_l_avg + rpm_l;
        }
        if(rpm_l != 0 || rpm_r !=0) {
          ///telem.print("(T) AVERAGE (L/R):  "); telem.print(rpm_l_avg/rpm_l_index); 
          //telem.print("    "); telem.println(rpm_r_avg/rpm_r_index);
          //telem.println();
          telem << "(T) Average (L/R):  " << rpm_l_avg/rpm_l_index << " / " << rpm_r_avg/rpm_r_index << endl;
        }
        
        read_sensors();
        oneSensorCycle();
        if(obs_array[3] == 1 || obs_array[0] == 1 || obs_array[1] == 1 || obs_array[2] == 1) {
          brake();
          return;
        }
      }
     }
   }
   
  if(obs_array[0] == 1  && obs_array[1] == 0  && obs_array[2] == 0) {
       telem.println("33% Left Blocked");
       brake();
       body_rturn();
       delay(200);	  //was 1000,400.150 
       brake();
       return;
   }
  if(obs_array[0] == 0  && obs_array[1] == 0  && obs_array[2] == 1) {
       telem.println("33% Right Blocked");
       brake();
       body_lturn();
       delay(185);	//was 1000,400,135
       brake();
       return;
   }
  if(obs_array[0] == 1  && obs_array[1] == 1  && obs_array[2] == 0) {
       telem.println("50% Left Blocked");
       brake(); 
       body_rturn();
       delay(275);     //was 1500, 700, 225
       brake();
       return;
   }
  if(obs_array[0] == 0  && obs_array[1] == 1  && obs_array[2] == 1) {
       telem.println("50% Right Blocked");
       brake(); 
       body_lturn();
       delay(275);	 //was 1500, 225	   
       brake();
       return;
   }
  if(obs_array[0] == 0  && obs_array[1] == 1  && obs_array[2] == 0) {
       telem.println("Middle Blocked");
       brake(); 
       moveBackward();
       delay(200);  
       brake();
       return;
   } 
   
}


