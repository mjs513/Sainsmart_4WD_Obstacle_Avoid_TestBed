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


void decide_direction_head() {
  brake();

  head_distance();
  
  if(obs_array[0] == 1 && obs_array[1] == 1 && obs_array[2] == 1 && obs_array[3] == 1 && obs_array[4] == 1) {
       //telem.println("All Directions Blocked - Rebound action");
       moveBackward();
       delay(500);	   
       return;
   }
   
  /*if(cm_head[0] > cm_head[4] &&  cm_head[0] > cm_head[2] ) {
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
 */
}

