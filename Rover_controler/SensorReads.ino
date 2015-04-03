void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  //for (uint8_t i = 0; i < SONAR_NUM; i++) {
  //  telem.print(i);
  //  telem.print("=");
  //  telem.print(cm[i]);
  //  telem.print("cm ");
  // }
  //telem.println();
  
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if(cm[i] < obsDist) {
      obs_array[i] = 1;
    } else {
      obs_array[i] = 0;} 
    //telem.print(obs_array[i]);
    //telem.print("  ");
  }
  //telem.println();
  
}

void read_sensors() {
    cm[0] = 0;  
    unsigned int uS = sonarll.ping();
    cm[0] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);	
	
    cm[1] = 0;  
    uS = sonarlc.ping();
    cm[1] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);	
	
    cm[2] = 0;  
    uS = sonarlr.ping();
    cm[2] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);

    cm[3] = 0;  
    uS = sonarhd.ping();
    cm[3] = uS / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);
    
    compass.read();
    aheading = compass.heading();
    yamartino.add(aheading);
    aheading = yamartino.averageHeading();
    //telem.print("Average Wind Direction: ");
    //telem.println(yamartino.averageHeading());
    //telem.println(aheading);
    //telem.println("");		

}


void head_distance() {
    headservo.write(head_lft);
    delay(100);
    cm_head[0]=sonarhd.ping() / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);
    
    headservo.write(head_ldiag);
    delay(100);
    cm_head[1]=sonarhd.ping() / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);
    
    headservo.write(head_fwd);
    delay(100);
    cm_head[2]=sonarhd.ping() / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);
    
    headservo.write(head_rdiag);
    delay(100);
    cm_head[3]= sonarhd.ping() / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);
    
    headservo.write(head_rt);
    delay(100);
    cm_head[4]=sonarhd.ping() / US_ROUNDTRIP_CM;
    delay(PING_INTERVAL);

    headservo.write(head_fwd);

  for (uint8_t i = 0; i < 5; i++) {
    telem.print(i);
    telem.print("=");
    telem.print(cm_head[i]);
    telem.print("cm Head");
  }
  telem.println();
  
  decide_direction_head();
}


