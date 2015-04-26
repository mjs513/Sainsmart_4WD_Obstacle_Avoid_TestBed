
float yar_heading, new_heading, angle_delta;

void Select_Direction() {
    telem << endl << endl << "************************************" << endl;
    telem << "Entering Select Direction Function" << endl;
    //local variables
    int i, start, left, delay_time;
    float fit_time, new_angle, Delta_Angle, angle_delta;
    float numer, denom;
    unsigned int sonar_dist[N+1], Hist[N+1];
    
    std::vector<std::pair<int,int> > border;
    std::vector<float> Candidate_Angle;
    std::pair<int,int> new_border;
    std::vector<std::pair<float,float> > rebound; 
 
    // Rotate head by N angles defined in Constants.h
    // default is 12 sectors and 15 Degrees from 0-180 degrees
    //
    for(i =0; i <= N; i++)     // goes from 180 degrees to 0 degrees 
    {
      headservo.write(i*angle);               // tell servo to go to position in variable 'pos' 
      delay(550);                             // waits 15ms for the servo to reach the position 

      //Read head ultrasonic sensor  
      unsigned int uS = sonarhd.ping(); 
      sonar_dist[i] = uS / US_ROUNDTRIP_CM;
      if(sonar_dist[i] == 0) sonar_dist[i] = 500;

      if(sonar_dist[i] <= obsDist) 
        Hist[i] = 1;
       else
        Hist[i] = 0;
        //telem <<  Hist[i] << ", " << sonar_dist[i] << endl;   
        telem << sonar_dist[i] << ", ";  
      delay(PING_INTERVAL); 
    } 
    telem << endl;
  
    /////////////////////////////////////////////////// 
    // only look at the forward 180deg for first obstacle.
    for(i = 0;i <= N;i++) 
    {
      if (Hist[i] == 1) 
      {
          start = i;
          break;
      }
    }
    
    //
    // Find the left and right borders of each opening
    //
    border.clear();

    //telem << "START: " << start << endl;
    start = 0;
    left = 1;
    for(i=start;i<=(start+N);i++) {
      if ((Hist[i % (N+1)] == 0) && (left)) {
        new_border.first = (i % (N+1)) * angle;
        left = 0;
        telem << "A-BORDER: " << new_border.first << endl;
      }

      if ((Hist[i % (N+1)] == 1) && (!left)) {
        new_border.second = ((i % (N+1)) - 1) * angle;
        border.push_back(new_border);
        left = 1;
        telem << "B-BORDER: " << new_border.second << endl;
      }
    }
    
    if(left == 0){
      new_border.second = (N) * angle;
      border.push_back(new_border);
      left = 1;
      telem << "B-BORDER: " << new_border.second << endl;
    }
   
    //
    // Consider each opening
    //
    int angle_cnt = 0;
    for(i=0;i < (int)border.size();i++) 
    {
      telem << "BORDER: " << border[i].first << " ,   " << border[i].second << endl;
      Delta_Angle = border[i].second - border[i].first + angle;
      //telem << "DELTA ANGLE: " << Delta_Angle << endl << endl;
    
      if (fabs(Delta_Angle) < 60) 
      {
        // narrow opening: aim for the centre
        new_angle = border[i].first + (border[i].second - border[i].first) / 2.0;
        //telem << "DELTA ANGLE: " << Delta_Angle << endl;
        telem << "Narrow Opening (New Angle):  " << new_angle << endl;
        Candidate_Angle.push_back(new_angle);
      } else {
        new_angle =  + (border[i].second - border[i].first) / 2.0;
        telem << "Wide Opening (Center):  " << new_angle << endl;
        Candidate_Angle.push_back(new_angle);
        
        //Bubble rebound for opening   
        numer = 0;  denom =0;
        //telem << "a-BORDER: " << border[i].first << " ,   " << border[i].second << endl;
        int start_index = border[i].first/angle;
        int end_index = border[i].second/angle;
        
        telem << start_index << ", " << end_index << endl;
        for(int i=start_index; i <= end_index; i++) {
          numer = numer + radians(angle*i)*sonar_dist[i];
          denom = denom + sonar_dist[i];
        }
        
        rebound[angle_cnt].second = denom / (end_index-start_index+1);
        rebound[angle_cnt].first = degrees(numer/denom);
        telem << "REBOUND ANGLE = " << rebound[angle_cnt].first << " degrees" << endl;
        telem << "Average Distance: " << rebound[angle_cnt].second <<endl;  
        telem << endl;
        angle_cnt = angle_cnt + 1;
      }
    }

    headservo.write(head_fwd);
    
    //Best angle based on highest average value of sensor distances
    //for opening
    int maxIndex = 0;
    float maxa = rebound[maxIndex].second;
    for (int i=1; i < angle_cnt; i++){
      if (maxa < rebound[i].second){
        maxa = rebound[i].second;
        maxIndex = i;
      }
    }
   
    float rebound_angle = rebound[maxIndex].first - 90;
    telem << "Best Angle:  " << rebound_angle << endl;

    compass_update(); 
    telem << "Current heading: " << yar_heading << endl; 
  
    float new_heading = yar_heading + (rebound_angle);
    telem << "Actual calc for new heading: " << new_heading << endl;
  
    if(new_heading > 360.0f)
       new_heading -= 360.0f;
    else if(new_heading < 0.0f)
      new_heading = 360.0f - new_heading;
  
    boolean clockwise;
    clockwise = true;
  
    angle_delta = abs(new_heading - yar_heading);
    clockwise = (rebound_angle >= 0.0f);
  
    telem << "New Heading: " << new_heading << "  Clockwise: " << clockwise << endl;
    telem << "Angle delta: " << angle_delta << endl;
    telem << endl;

    if(clockwise) {
      fit_time = 0.000006*pow(rebound_angle,3)-0.0081*pow(rebound_angle,2)+7.1036*rebound_angle+84.232;
      delay_time = ceil(fit_time);
      telem << "Curve Fit (CW): " << fit_time << endl;
      if(delay_time < 50) delay_time = 50;
      body_rturn(); 
    } else {
      fit_time = -0.00001*pow(rebound_angle,3)-0.015*pow(rebound_angle,2)-9.7223*rebound_angle+32.217;
      delay_time = ceil(fit_time);
      telem << "Curve Fit (CCW): " << fit_time << endl;      
      if(delay_time < 50) delay_time = 50;
      body_lturn();
    }
  
    telem << "Turn Time: " << delay_time << endl;
    delay(delay_time);
    brake();
    
    compass_update();
    telem << "New Heading: " << yar_heading << endl;
        
}


  
