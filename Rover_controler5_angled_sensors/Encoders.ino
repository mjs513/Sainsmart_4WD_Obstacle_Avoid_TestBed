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

//**********************************************************
//ecoder rpm read
void encoder_l() { 
   if (half_revolutions_l >= 10) { //was 40, 20
     //Update RPM every 20 counts, increase this for better RPM resolution,
     //decrease for faster update
     rpm_l = 1000/(millis() - timeold_l)*half_revolutions_l; //3.75 Dagu
     timeold_l = millis();
     half_revolutions_l = 0;
     //telem.println();telem.println("Left:  ");telem.println(rpm_l,DEC);telem.println();
   }
}

void encoder_r() { 
    if (half_revolutions_r >= 10) { 
     //Update RPM every 20 counts, increase this for better RPM resolution,
     //decrease for faster update
     rpm_r = 1000/(millis() - timeold_r)*half_revolutions_r;
     timeold_r = millis();
     half_revolutions_r = 0;
     //telem.println();telem.println("Right:  ");telem.println(rpm_r,DEC);telem.println();
   }
}

 void rpm_fun_l()
 {
   half_revolutions_l++;
   //Each rotation, this interrupt function is run twice
 }

 void rpm_fun_r()
 {
   half_revolutions_r++;
   //Each rotation, this interrupt function is run twice
 }


//**********************************************************
//IsTime() function - David Fowler, AKA uCHobby, http://www.uchobby.com 01/21/2012

#define TIMECTL_MAXTICKS  4294967295L
#define TIMECTL_INIT      0

int IsTime(unsigned long *timeMark, unsigned long timeInterval){
  unsigned long timeCurrent;
  unsigned long timeElapsed;
  int result=false;
  
  timeCurrent=millis();
  if(timeCurrent<*timeMark) {  //Rollover detected
    timeElapsed=(TIMECTL_MAXTICKS-*timeMark)+timeCurrent;  //elapsed=all the ticks to overflow + all the ticks since overflow
  }
  else {
    timeElapsed=timeCurrent-*timeMark;  
  }

  if(timeElapsed>=timeInterval) {
    *timeMark=timeCurrent;
    result=true;
  }
  return(result);  
}
