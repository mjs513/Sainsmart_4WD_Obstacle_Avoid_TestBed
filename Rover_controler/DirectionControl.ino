
void decide_direction() {
  if(obs_array[3] == 1 && obs_array[0] == 0 && obs_array[1] == 0 && obs_array[2] == 0) {
       telem.println("OBSTACLE HEIGHT - STOP or REVERSE");
       head_distance(); 
       return;
     }
  if(obs_array[0] == 1 && obs_array[2] == 1) {
       telem.println("STOP or Reverse ");
       brake();
       head_distance(); 
       return;
   } 
   
  if(obs_array[3] == 0 && obs_array[0] == 0 && obs_array[1] == 0 && obs_array[2] == 0) {
      telem.println("NO OBSTACLES");
      moveForward(motorSpeed);
      currentTime = millis();
      while(!IsTime(&currentTime, interval)){
         encoder_l();
         encoder_r();
      }
      brake();
      return;
   }
   
  if(obs_array[0] == 1  && obs_array[1] == 0  && obs_array[2] == 0) {
       telem.println("33% Left Blocked");
       brake();
       body_rturn(turnSpeed);
       delay(1000);	   
       brake();
       return;
   }
  if(obs_array[0] == 0  && obs_array[1] == 0  && obs_array[2] == 1) {
       telem.println("33% Right Blocked");
       brake();
       body_lturn(turnSpeed);
       delay(1000);		   
       brake();
       return;
   }
  if(obs_array[0] == 1  && obs_array[1] == 1  && obs_array[2] == 0) {
       telem.println("50% Left Blocked");
       brake(); 
       body_rturn(turnSpeed);
       delay(1500);	   
       brake();
       return;
   }
  if(obs_array[0] == 0  && obs_array[1] == 1  && obs_array[2] == 1) {
       telem.println("50% Right Blocked");
       brake(); 
       body_lturn(turnSpeed);
       delay(1500);	 	   
       brake();
       return;
   }  
}


