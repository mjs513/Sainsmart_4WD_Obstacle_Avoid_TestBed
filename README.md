# Sainsmart_4WD_Obstacle_Avoid_TestBed
This is a work in progress on developing methods for obstacle avoidance, wheel encoders, and other methodologies.  It is based on code from several sources which I will incorporate as I go.

Code based on example code from following:
```
 Sainsmart Obstacle Avoidance Robot:
    http://www.mkme.org/index.php/arduino-sainsmart-4wd-robot/
    https://github.com/gmossy/Sainsmart-4WD-Robot
 Wheel Encoders - the DAGU Simple Encoder Kit
    http://www.bajdi.com/adding-encoders-to-those-cheap-yellow-motors/
    http://letsmakerobots.com/node/38636
    http://playground.arduino.cc/Main/ReadingRPM
    http://elimelecsarduinoprojects.blogspot.com/2013/06/measure-rpms-arduino.html 
  Compass Averaging
    Yamartino Library, Christopher Baker  https://github.com/SAIC-ATS/Algorithms.git
  Obstacle avoidance approaches
    http://homepages.engineering.auckland.ac.nz/~pxu012/mechatronics2013/group7/index.html
    http://homepages.engineering.auckland.ac.nz/~pxu012/mechatronics2013/group7/software.html
  IR Sensing
    http://letsmakerobots.com/node/40502
    http://adhocnode.com/arduino-irrecv-module/
    http://arduino-info.wikispaces.com/IR-RemoteControl
```

More to follow as I add PID controls and Bubble rebound algorithm for obstacle avoidance
----------------------------------------------------------
4-20-15
1) Added license
2) Embedded test code for bubble rebound which is still a work in progress
3) Added code to increse I2C bus speed to 400khz
4) Added code to change default altimu10 v3 default settings
5) In process of generating calibration code for time delay versus degrees for compass code - will embed charts shortly.
6) Changed motor control code so you can have different speeds for left and right turn

4-3-15

1) adjusted left right motor speeds based on encoder to keep it going as straight as possible
2) compass is working.  Actually using an Altimu-10 v3 IMU, but just getting compass readings.  Eventually will expand to get attidude as well.
3) Incorporated the Streaming.h library so I can use standared c++ cout methods for printing.  Now have to fix remaing code to make use of it.
4) Next step is compass control and the bubble band algorithm. Also have to fix back up so it will backup straight and adjust some of the delays since I slowed the motor up a little bit.


3-31-15

Moved away from the Sainsmart 4wd test bed.  Just not enough torgue if I want to run it on a carpet. Was even slipping on a un-even kitchen floor.  Used a differential platform that I picked up off ebay.  Built a platform, took the electronics from the sainsmart and mounted it on board and had a new platform to experiment with.   As a result was able to get the code working so that it has now stopped crashing into corners, however, need to work on two situations: 1) getting caught on a corner (have to take robot width into account and (2) picking the best direction to move in.
