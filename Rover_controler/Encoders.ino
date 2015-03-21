
//**********************************************************
//ecoder rpm read
void encoder_l() { 
   if (half_revolutions_l >= 20) { 
     //Update RPM every 20 counts, increase this for better RPM resolution,
     //decrease for faster update
     rpm_l = 1000/(millis() - timeold_l)*half_revolutions_l; //3.75 Dagu
     timeold_l = millis();
     half_revolutions_l = 0;
     telem.print("Left  ");telem.println(rpm_l,DEC);
   }
}

void encoder_r() { 
    if (half_revolutions_r >= 20) { 
     //Update RPM every 20 counts, increase this for better RPM resolution,
     //decrease for faster update
     rpm_r = 1000/(millis() - timeold_r)*half_revolutions_r;
     timeold_r = millis();
     half_revolutions_r = 0;
     telem.print("Right  ");telem.println(rpm_r,DEC);
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
