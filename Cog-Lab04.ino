/* Cog-Lab04
 *  ME425-Mobile Robotics
 *  Lab 4 - Wall Following via Proportional-Derviative (PD) Feedback Control
 *  
 *  Description:
 *  This code is used to control the robot to randomly wander until it senses a wall. Once it senses a wall on either
 *  side of it, it follows that wall.  If it senses a wall on each side, it moves to the center of the two walls and
 *  moves forward while staying in the center.  The robot can navigate doorways, turn corners and continue to follow
 *  a wall, and avoid obstacles placed next to walls.  when the robot is unable to sense a wall on either side, it
 *  resorts back to randomly wandering.
 *  
 *  Authors:
 *  Chris Collinsworth
 *  Jordan Patterson
 *  
 *  Latest Revision Date:
 *  05/18/2018
 ************************************************************************
 */

// Include the necessary libraries
#include <ArduinoRobot.h>
#include <Wire.h>
#include <SPI.h>
#include <Math.h>

// MakeCircle Setup
#define LeftSonarPin TKD2
#define RightSonarPin TKD0
#define FrontIRPin TK2
#define BackIRPin TK6

//sensor values
float Leftvalue = 0;
float Rightvalue = 0;
float Frontvalue = 0;

// For converting functions
float IRvalue = 0;
float Sonarvalue = 0;
// IR distances
float IRdistance = 0;
float Frontconv = 0;

// Sonar distances
float Sonardistance = 0;
float Leftconv = 0;
float Rightconv = 0;


//distance range
double maxDistance = 10; //8 in max range for sensor reading
double deadZoneMin = 4;
double deadZoneMax = 6;
double theta = 0;

// PD Variables
float errorLeft = 0;
float errorRight = 0;
float derivativeRight = 0;
float derivativeLeft = 0;
float leftOutput = 0;
float rightOutput = 0;
float leftErrorPrior = 0;
float rightErrorPrior = 0;
int time1 = 0;

// PD constants
double Kp = 2;
float Kd = 0.005;

// Desired sensor value
float sensorVal_des = 5;

// Motor speeds
double slow_spd = 75;
double med_spd  = 100;
double fast_spd = 175;

//enum setup for modes of robot
enum STATE {
  Wander,
  LeftFollow,
  RightFollow,
  CenterFollow
};
//set the default state
int STATE_current = Wander;

void setup() {
  // initialize the Robot, display, and speaker
  Robot.begin();
  Robot.beginTFT();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
  Robot.text("State:", 5, 5);
}
//loop through reading the sensor and setting the state in the state machine
void loop() {
  ReadSensor();
  SetState();
}
//define robot move funciton for PD control
void RobotMove(float leftOutput, float rightOutput) {
  Robot.motorsWrite(med_spd + leftOutput, med_spd + rightOutput);
}
//derivative control for left wall follow
void PD_Left() {
  while (STATE_current = LeftFollow) {
    // read sensor and move robot 
    ReadSensor();
    RobotMove(leftOutput, rightOutput);
    //calculate error from desired value and the actual distance
    errorLeft = sensorVal_des - Leftconv;
    //if robot is in dead (4-6 in)
    if (abs(errorLeft) <= (deadZoneMax - sensorVal_des)) {
      ReadSensor();
      leftOutput = 0;
    }
    //if the wall is lost move forward, turn left, then move forward again 
    else if (Leftconv > 13) {
      delay(250);
      Robot.motorsWrite(0, 0);
      delay(100);
      GoToAngle(80);
      Robot.motorsWrite(med_spd, med_spd);
      delay(2700);
      Robot.motorsWrite(0, 0);
      delay(100);
      //if doesnt see wall again go into random
      ReadSensor();
      if (Leftconv > 13) {
        SetState();
      }
    }
    //if the front sensor is too close, back up and turn right
    else if (Frontconv < 5) {
      Robot.motorsWrite(0, 0);
      delay(100);
      Robot.motorsWrite(-75, -75);
      delay(500);
      GoToAngle(-45);
    }
    //if right distance within range, go to set state and it will go into center
    else if (Rightconv < 10) {
      SetState();
    }
    //otherwise adjust the left wheel to get back on track
    else {
      ReadSensor();
      int newtime = millis();
      int dt = newtime - time1;
      derivativeLeft = (leftErrorPrior - errorLeft) / dt ;
      leftOutput = (Kp * errorLeft) + (Kd * derivativeLeft);
      leftErrorPrior = errorLeft;
      time1 = newtime;
    }
  }
}
//derivative control for right wall follow
void PD_Right() {
  while (STATE_current = RightFollow) {
    // read sensor and move robot 
    ReadSensor();
    RobotMove(leftOutput, rightOutput);
    //calculate error from desired value and the actual distance
    errorRight = sensorVal_des - Rightconv;
    //if robot is in dead (4-6 in)
    if (abs(errorRight) <= (deadZoneMax - sensorVal_des)) {
      ReadSensor();
      rightOutput = 0;
    }
    //if the wall is lost move forward, turn right, then move forward again 
    else if (Rightconv > 13) {
      delay(250);
      Robot.motorsWrite(0, 0);
      delay(100);
      GoToAngle(-80);
      Robot.motorsWrite(med_spd, med_spd);
      delay(2700);
      Robot.motorsWrite(0, 0);
      delay(100);
      //if doesnt see wall again go into random
      ReadSensor();
      if (Rightconv > 13) {
        SetState();
      }
    }
    //if the front sensor is too close, back up and turn right
    else if (Frontconv < 5) {
      Robot.motorsWrite(0, 0);
      delay(100);
      Robot.motorsWrite(-75, -75);
      delay(500);
      GoToAngle(45);
    }
    //if left distance within range, go to set state and it will go into center
    else if (Leftconv < 10) {
      SetState();
    }
    //otherwise adjust the left wheel to get back on track
    else {
      ReadSensor();
      int newtime = millis();
      float dt = newtime - time1;
      derivativeRight = (rightErrorPrior - errorRight) / dt ;
      rightOutput = (Kp * errorRight) + (Kd * derivativeRight);
      rightErrorPrior = errorRight;
      time1 = newtime;
    }
  }
}
//follow hallway
void PD_Center() {
  while (STATE_current = CenterFollow) {
    ReadSensor();
    RobotMove(leftOutput, rightOutput);
    errorRight = sensorVal_des - Rightconv;
    errorLeft = sensorVal_des - Leftconv;
    //if robot is in dead (4-6 in)
    if ((abs(errorRight) <= (deadZoneMax - sensorVal_des)) && (abs(errorLeft) <= (deadZoneMax - sensorVal_des))) {
      ReadSensor();
      rightOutput = 0;
      leftOutput = 0;
    }
    //if either of the sides go beyond range, go to state machine
    else if ((Rightconv > 13) || (Leftconv > 13)) {
      SetState();
    }
    //otherwise adjust the wheels to get back in the center
    else {
      ReadSensor();
      int newtime = millis();
      float dt = newtime - time1;
      //right control
      derivativeRight = (rightErrorPrior - errorRight) / dt ;
      rightOutput = (Kp * errorRight) + (Kd * derivativeRight);
      rightErrorPrior = errorRight;
      //left control
      derivativeLeft = (leftErrorPrior - errorLeft) / dt ;
      leftOutput = (Kp * errorLeft) + (Kd * derivativeLeft);
      leftErrorPrior = errorLeft;

      time1 = newtime;
    }
  }
}

//read sensors for following
void ReadSensor(void) {
  Frontvalue = Robot.analogRead(FrontIRPin);
  //convert distances to inches
  Frontconv = FrontIRConvertToInches(Frontvalue);


  //left sonar reading
  pinMode(LeftSonarPin, OUTPUT);//set the PING pin as an output
  Robot.digitalWrite(LeftSonarPin, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  Robot.digitalWrite(LeftSonarPin, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  Robot.digitalWrite(LeftSonarPin, LOW);//set pin low first again
  pinMode(LeftSonarPin, INPUT);//set pin as input with duration as reception time
  Leftvalue = pulseIn(LeftSonarPin, HIGH);//measures how long the pin is high
  Leftconv = LeftSonarConvertToInches(Leftvalue);

  //right sonar reading
  pinMode(RightSonarPin, OUTPUT);//set the PING pin as an output
  Robot.digitalWrite(RightSonarPin, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  Robot.digitalWrite(RightSonarPin, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  Robot.digitalWrite(RightSonarPin, LOW);//set pin low first again
  pinMode(RightSonarPin, INPUT);//set pin as input with duration as reception time
  Rightvalue = pulseIn(RightSonarPin, HIGH);//measures how long the pin is high
  Rightconv = RightSonarConvertToInches(Rightvalue);
}
//random wander for when ranges are too large
void RandomWander() {
  int LEFT_MOTOR_SPEED = 0;       //
  int RIGHT_MOTOR_SPEED = 0;       //
  //while range is beyond 10 randomly wander
  while (Rightconv > 10 && Leftconv > 10) {
    ReadSensor(); //read the sensors
    //randomly move the robot at different motor speeds
    Robot.text("wander", 5, 65);
    LEFT_MOTOR_SPEED = random(-55, 200);
    RIGHT_MOTOR_SPEED = random(-55, 200);
    Robot.motorsWrite(LEFT_MOTOR_SPEED, RIGHT_MOTOR_SPEED);
    //if any of the sensor seadings are below the max distance, stop the robot and change state
    if ((Leftconv < maxDistance) || (Rightconv < maxDistance)) {
      Robot.motorsWrite(0,0);
      delay(150);
      SetState();
    }
    //if front sensor is too close back up the robot and turn 
    else if (Frontconv < 5) {
      Robot.motorsWrite(0, 0);
      delay(100);
      Robot.motorsWrite(-75, -75);
      delay(500);
      GoToAngle(45);
    }
    delay(250);
  }
}
void updateLCD() {
  // clear the LCD screen
  Robot.fill(255, 255, 255);
  Robot.stroke(255, 255, 255);
  Robot.rect(5, 15, 40, 10);
  //printing the current state to the LCD screen
  switch (STATE_current) {
    case Wander:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Wander", 5, 15);
      break;
    case LeftFollow:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Left", 5, 15);
      break;
    case RightFollow:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Right", 5, 15);
      break;
    case CenterFollow:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Center", 5, 15);
      break;
  }
}

void SetState() {
  //if both sensors are withing range go to center follow
  if (Leftconv < 10 && Rightconv < 10) {
    STATE_current = CenterFollow;
    updateLCD();
    PD_Center();
  }
  //if only left is broken, go to left follow
  else if (Leftconv < 10) {
    STATE_current = LeftFollow;
    updateLCD();
    PD_Left();
  }
  //if only right is broken, go to right follow
  else if (Rightconv < 10) {
    STATE_current = RightFollow;
    updateLCD();
    PD_Right();
  }
  //if both are beyond the range go to random wander
  else if (Rightconv > 10 && Leftconv > 10) {
    STATE_current = Wander;
    updateLCD();
    RandomWander();
  }
}
//go to angle function for when front sensor is within range
void GoToAngle(float theta) { //rotates the robot a desired angle
  int turn_delay = 0;
  if (theta > 0) { //Positive theta rotates bot CCW
    Robot.motorsWrite(-fast_spd, fast_spd);
    Robot.stroke(0, 0, 0);
    Robot.text("Turning!", 5, 85); //tell user the bot is turning
    turn_delay = (theta + 18.833) / 0.3683; //delay based on excel sheet calculation of how far the robot turned at a specified delay

  }
  else if (theta < 0) { //negative theta rotates bot CW
    Robot.motorsWrite(fast_spd, -fast_spd);
    Robot.stroke(0, 0, 0);
    Robot.text("Turning!", 5, 85);
    turn_delay = (abs(theta) + 18.833) / 0.3683;

  }
  delay(turn_delay); //delay the movement to reach the desired angle
  Robot.stroke(255, 255, 255);
  Robot.text("Turning!", 5, 35);
  Robot.motorsWrite(0, 0);

  delay(250);
}

//functions for converting values to distances. 
float FrontIRConvertToInches(float IRValue) {
  IRdistance = 4122.3 * (pow(IRValue, -1.121));
  return IRdistance;
}
float RightSonarConvertToInches(float Sonarvalue) {
  Sonardistance = (0.0068 * Sonarvalue) - 0.7455;
  return Sonardistance;
}
float LeftSonarConvertToInches(float Sonarvalue) {
  Sonardistance = (0.0069 * Sonarvalue) - 0.6614;
  return Sonardistance;
}







