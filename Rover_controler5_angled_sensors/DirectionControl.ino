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

void decide_direction() {
   // Start with the assumption that we can go straight.
   if(cm[3] > fowardheadThreshold && cm[0] > sideSensorThreshold && 
                 cm[1] > lcThreshold && cm[2] > sideSensorThreshold)
           nextMove = "Straight";

   // Reset the closest obstacle distance to a large value.
   minDistance = 10000;

   // Do any of the front facing range sensors detect an obstacle closer than their
   // threshold?  If so, then prepare to turn left or right.
   if (cm[3] < fowardheadThreshold ||
       cm[1] < lcThreshold )
   {
      nextMove = "LeftRight";
   }

   // What about the two sideways looking IR detectors?
   if (cm[0] < sideSensorThreshold && cm[2] <  sideSensorThreshold)
   {
       nextMove = "LeftRight";
   }
   else if (cm[2] < sideSensorThreshold)
   {
      nextMove = "Right";
   }
   else if(cm[0] < sideSensorThreshold)
   {
      nextMove = "Left";
   }

   // How close is the closest object in front of us?  Note how we subtract
   // 4" from the front center IR sensor (under the robot) and 5" from the
   // the tower IR sensor.  This is because these two sensors are offset by
   // these distances back from the front of the robot.
   //minDistance = min(obs_array[0], obs_array[2], obs_array[1], obs_array[3]);
   minIndex = 0;
   float mina = obs_array[minIndex];
   for (int i=1; i < 4; i++){
      if (mina > cm[i]){
          mina = cm[i];
          minIndex = i;
       }
   }
   minDistance = cm[minIndex];
   
   // If the closest object is too close, then get ready to back up.
   if (minDistance < backupSensorThreshold) nextMove = "Backup";
  
   // If we can't go forward, stop and take the appropriate evasive action.
   if (nextMove != "Straight")
   {
       brake();
       if (nextMove == "Backup")
       {
          // Keep track of the last action.
          lastMove = "Backup";
          moveBackward();
          delay(250);
          coastBrake();
          delay(500);
          Select_Direction();
          if(clockwise == true) {
            lastMove = "Left";
            nextMove = "Left";
          } else {
            lastMove = "Right";
            nextMove = "Right"; }
          return;
        } else {
          // Make sure we don't oscillate back and forth in a corner.
          if (lastMove == "Left" || lastMove == "Right") 
                                             nextMove = lastMove;
          
          if (nextMove == "Left")  
                          nextTurn = -1;
          if (nextMove == "Right") 
                          nextTurn = 1;
          if (nextMove == "LeftRight") {
              if(random(-1,1) >= 0 ){ 
                  nextTurn = 1;
	      } else {
		  nextTurn = -1; }
	  }
          //Random left or right.

          // Keep track of the last turn.
          if (nextTurn == -1) 
              lastMove = "Left";
          else 
              lastMove = "Right";

	  if (nextTurn == -1) {
	      body_lturn();
	      delay(325);     //was 1500, 700, 225 - calc at 275 change to 325
	      brake();
	   } else {
	      body_rturn();
	      delay(325);     //was 1500, 700, 225 - calc at 275 change to 325
	      brake(); }
          return;
        }
   } else {
       // If no obstacles are detected close by, keep going straight ahead.
       //moveForward();
       lastMove = "Straight";
       nextMove = lastMove;

       while(cm[3] > fowardheadThreshold && cm[0] > sideSensorThreshold && 
                 cm[1] > lcThreshold && cm[2] > sideSensorThreshold) {

          moveForward();
           
          rpm_r_index = 0;  rpm_l_index = 0;
          rpm_r_avg = 0;    rpm_l_avg = 0;
          //currentTime = millis();
        
          //while(!IsTime(&currentTime, interval))
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
          telem << "(DC) Average (L/R):  " << rpm_l_avg/rpm_l_index << " / " << rpm_r_avg/rpm_r_index << endl;
          }
          
          read_sensors();   
          oneSensorCycle();          
       }    
      brake();
    }
   return;
}   

