  
//******************   moveForward *******************
//void moveForward(int Speed)
void moveForward()
{
   // int motorSpeed);  // change the 15 to the Speed variable, and put Speed int the function call command arguments.
    //also add delayTime for example like this:   moveForward(int delayTime, int motorSpeed)
    
    //motorA(2, Speed);  //have motor A turn clockwise at % speed, , call motor control method
    //motorB(2, Speed);  //have motor B turn clockwise at % speed
    motorA(2, motorSpeed_right);  //have motor A turn clockwise at % speed, , call motor control method
    motorB(2, motorSpeed_left);  //have motor B turn clockwise at % speed
    telem.println("Forward");
}

//void moveBackward(int Speed)
void moveBackward()
{
    //motorA(1, Speed);  //have motor A turn counterclockwise 
    //motorB(1, Speed);  //have motor B turn counterclockwise
    motorA(1, motorSpeed_right);  //have motor A turn counterclockwise 
    motorB(1, motorSpeed_left);  //have motor B turn counterclockwise
    telem.println("Backward");
}
void body_rturn(int Speed)
{
   motorA(1, Speed);  //have motor A turn counterclockwise 
   motorB(2, Speed);  //have motor B turn clockwise 
   telem.println("Right");
}
   void body_lturn(int Speed)
{
   motorA(2, Speed);  //have motor A turn clockwise
   motorB(1, Speed);  //have motor B turn counterclockwise 
   telem.println("Left");
}

void brake()
{
    motorA(3, 100);  //brake motor A with 100% braking power, call motor control method
    motorB(3, 100);  //brake motor B with 100% braking power, call motor control method
    telem.println("Brake");
}

//******************   Motor A control   *******************
void motorA(int mode, int percent)
{
 // Method that will accept mode and speed in percentage.  
 // mode: The 3 modes are 0) coast  1) Clockwise motor  2) Counter-clockwise 3) brake
 // percent: The speed of the motor in percentage value for the PWM.
  //change the percentage range of 0 -> 100 into the PWM
  //range of 0 -> 255 using the map function
  int duty = map(percent, 0, 100, 0, 255);
  
  switch(mode)
  {
    case 0:  //disable/coast
      digitalWrite(ENA, LOW);  //set enable low to disable A
      telem.println("Disable/coast A");
      break;
      
    case 1:  //turn clockwise
      //setting IN1 high connects motor lead 1 to +voltage
      digitalWrite(IN1, HIGH);   
      
      //setting IN2 low connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      telem.println("turn motor A clockwise");
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to +voltage
      digitalWrite(IN2, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENA, duty);  
      telem.println("turn motor A counter-clockwise");
      
      break;
      
    case 3:  //brake motor
      //setting IN1 low connects motor lead 1 to ground
      digitalWrite(IN1, LOW);   
      
      //setting IN2 high connects motor lead 2 to ground
      digitalWrite(IN2, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENA, duty);  
      telem.println("brake motor A");
      
      break;
  }
}
//**********************************************************
//******************   Motor B control   *******************
  void motorB(int mode, int percent)
{
  //change the percentage range of 0 -> 100 into the PWM
  //range of 0 -> 255 using the map function
  int duty = map(percent, 0, 100, 0, 255);
  
  switch(mode)
  {
    case 0:  //disable/coast
      digitalWrite(ENB, LOW);  //set enable low to disable B
      telem.println("Disable/coast B");
      break;
      
    case 1:  //turn clockwise
      //setting IN3 high connects motor lead 1 to +voltage
      digitalWrite(IN3, HIGH);   
      
      //setting IN4 low connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty); 
           telem.println("turn motor B clockwise"); 
      
      break;
      
    case 2:  //turn counter-clockwise
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to +voltage
      digitalWrite(IN4, HIGH);  
      
      //use pwm to control motor speed through enable pin
      analogWrite(ENB, duty);  
       telem.println("turn motor B counter-clockwise");
      
      break;
      
    case 3:  //brake motor
      //setting IN3 low connects motor lead 1 to ground
      digitalWrite(IN3, LOW);   
      
      //setting IN4 high connects motor lead 2 to ground
      digitalWrite(IN4, LOW);  
      
      //use pwm to control motor braking power 
      //through enable pin
      analogWrite(ENB, duty);  
      telem.println("brake motor B");      
      break;
  }
}


