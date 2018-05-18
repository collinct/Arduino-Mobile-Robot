/* Cog-Lab06
 *  ME425-Mobile Robotics
 *  Lab 6 - Homing & Docking Via Hybrid Control; Basic Path Planning
 *  
 *  Description:
 *  This code is used to control the robot to follow a wall until it senses a light beacon.  Once a light beacon
 *  is sensed, the robot leaves the wall and moves toward the light source to "dock" with it (stop before hitting it).
 *  While moving toward the light beacon, the robot uses obstacle avoidance to avoid obstacles and also uses a path
 *  planning algorithm to plan a path for the robot to follow after reaching the light beacon so it can return to
 *  the wall.
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

// MakeCircle Setup
#define LeftPhotoPin TK3
#define LeftSonarPin TKD2
#define RightPhotoPin TK1
#define FrontIRPin TK2

// Sensor values
float Sonar = 0;
int Frontvalue = 0;
int Leftvalue = 0;
int Rightvalue = 0;
// Converteddistances
float Leftconv = 0;
float Rightconv = 0;
float Sonconv = 0;
//for converting functions
float IRvalue = 0;
float Sonarvalue = 0;
//IR distances

float Frontconv = 0;
// Motor speeds
double slow_spd = 75;
double med_spd  = 115;
double fast_spd = 175;
// Min and max values for photoresistors
int leftMin = 780;
int leftMax = 1023;
int rightMin = 850;
int rightMax = 920;
double speedMin = 150;
double speedMax = 225;
// PD Variables
float errorLeft = 0;
float derivativeLeft = 0;
float leftOutput = 0;
float rightOutput = 0;
float leftErrorPrior = 0;
int time1 = 0;
// Desired sensor value
float sensorVal_des = 5;
// PD constants
double Kp = 2;
float Kd = 0.005;

//avoid flag for the option of obstacle avoidance
boolean Avoid = false;
//array aetup
#define pathlength 20
int path[pathlength];
int index = 0;

//enum for the mode selection
enum STATE {
  Light,
  Wall,
  PathFrom
};
//default state is the Ready state
int STATE_current = Wall;
//int STATE_current = Light;

void setup() {
  // initialize the Robot and display
  Robot.begin();
  Robot.beginTFT();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
  Robot.text("State:", 5, 5);
  for (int i = 0; i < pathlength; i++) {
    path[i] = 0;
  }

}
void loop() {
  ReadSensor();
  //run the state machine
  //  Rightconv = map(Rightvalue, rightMin , rightMax, speedMin, speedMax);
  //  Serial.println(Leftvalue);
  SetState();
}


//derivative control for left wall follow
void PD_Left() {
  //distance range
  double maxDistance = 10; //8 in max range for sensor reading
  double deadZoneMin = 4;
  double deadZoneMax = 6;
  while (STATE_current == Wall) {
    // read sensor and move robot
    ReadSensor();
    RobotMove(leftOutput, rightOutput);
    //calculate error from desired value and the actual distance
    errorLeft = sensorVal_des - Sonconv;

    //    if (Rightvalue > rightMin + 10) {
    if ((Rightvalue > rightMin + 10) || (Leftvalue > leftMin + 10)) {
      Robot.text("Found it!", 5, 25);
      Robot.motorsWrite(0, 0);
      delay(100);
      STATE_current = Light;
      SetState();
      break;
    }
    //if robot is in dead (4-6 in)
    if (abs(errorLeft) <= (deadZoneMax - sensorVal_des)) {
      ReadSensor();
      leftOutput = 0;
      //      if (Rightvalue > rightMin + 10) {
      if ((Rightvalue > rightMin + 10) || (Leftvalue > leftMin + 10)) {
        Robot.text("Found it!", 5, 25);
        Robot.motorsWrite(0, 0);
        delay(100);
        STATE_current = Light;
        SetState();
        break;
      }
    }
    //if the wall is lost move forward, turn left, then move forward again
    else if (Sonconv > 13) {
      delay(250);
      Robot.motorsWrite(0, 0);
      delay(100);
      GoToAngle(80);
      Robot.motorsWrite(med_spd, med_spd);
      delay(2000);
      Robot.motorsWrite(0, 0);
      delay(100);
      //if doesnt see wall again go into random
      ReadSensor();
      //      if (Rightvalue > rightMin + 10)  {
      if ((Rightvalue > rightMin + 10) || (Leftvalue > leftMin + 10)) {
        Robot.text("Found it!", 5, 25);
        Robot.motorsWrite(0, 0);
        delay(100);
        STATE_current = Light;
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
void FollowLight(void) {
  while (STATE_current == Light) {
    //while in this state read sensors
    ReadSensor();
    //if the button_up is pressed again turn on obstacle avoidance
    if (Frontconv < 5) { //if the front sensor detects an obstacle
      Robot.motorsWrite(0, 0);
      Robot.text("At goal", 5, 75);
      GoToAngle(225);
      Robot.motorsWrite(0, 0);
      STATE_current = PathFrom;
      SetState();
    }
    else {
      //if both are smaller than the min then dont move the bot
      if ((Leftvalue < leftMin) && (Rightvalue < rightMin)) {
        Leftconv = med_spd;
        Rightconv = med_spd;
        path[index] = 3;
        Robot.text("none", 5, 45);
      }
      //if left value is detected and not right
      else if ((Leftvalue >= leftMin) && (Rightvalue < rightMin )) {
        //set left value to the minimum
        Leftvalue = leftMin;
        path[index] = 2;
        Robot.text("Found it!", 5, 55);
        Leftconv = map(Leftvalue, leftMin, leftMax, speedMin, speedMax);
        Rightconv = map(Rightvalue, rightMin, rightMax, speedMin, speedMax);
      }
      //if a right value is detected and not left
      else if ((Leftvalue < leftMin) && (Rightvalue >= rightMin )) {
        //set right value to the minimum
        Rightvalue = rightMin;
        path[index] = 1;
        Robot.text("Found it!", 5, 55);
        Leftconv = map(Leftvalue, leftMin, leftMax, speedMin, speedMax);
        Rightconv = map(Rightvalue, rightMin, rightMax, speedMin, speedMax);
      }
      else {
        //else just map the values
        Leftconv = map(Leftvalue, leftMin , leftMax, speedMin, speedMax);
        Rightconv = map(Rightvalue, rightMin , rightMax, speedMin, speedMax);
      }
      //      for (int i=0;i<10;i++){
      Serial.println(path[index]);
      //      }
      index++;
      delay(250);

      //move the bot based on the if statements' output
      Robot.motorsWrite(Rightconv, Leftconv);
      delay(250);
      Robot.motorsWrite(0, 0);
    }
  }
}
void PathBack() {
  for (int index = 0; index < pathlength; index++) {
    Serial.print(path[index]);
  }
  Serial.println("");
  while (STATE_current == PathFrom) {

    for (int i = pathlength - 1 ; i >= 0; i--) {
      //            Robot.debugPrint((path[i]),5,85);
      //      Serial.println(path[index]);

      if (path[i] == 3) { //go forward
        Robot.text("forward", 5, 45);
        
        Leftconv = med_spd;
        Rightconv = med_spd;
      }
      else if (path[i] == 1) { //if index was right go left this time
        Robot.text("left", 5, 55);
        
        Leftvalue = leftMin;
        Leftconv = map(Leftvalue, leftMin, leftMax, speedMin, speedMax);
        Rightconv = map(Rightvalue, rightMin, rightMax, speedMin, speedMax);
      }
      else if (path[i] == 2) { //if index was left go right this time
        Robot.text("right", 5, 65);
        
        Rightvalue = rightMin;
        Leftconv = map(Leftvalue, leftMin, leftMax, speedMin, speedMax);
        Rightconv = map(Rightvalue, rightMin, rightMax, speedMin, speedMax);
      }
      else if (path[i] == 0) { //if index was 0 do nothing
        
        Leftconv = 0;
        Rightconv = 0;
      }
      delay(250);

      //move the bot based on the if statements' output
      Robot.motorsWrite(Rightconv, Leftconv);
      delay(250);
      Robot.motorsWrite(0, 0);
      if (i == 0) {
//        Robot.motorsWrite(0, 0);
        while (i == 0) {
          ReadSensor();
//          Serial.println(Sonconv);
          if (Sonconv > 6 ){
            Serial.println("turning");
            GoToAngle(-45);
            delay(250);
          }
          else if (Sonconv < 6) {
            Serial.println("at wall");
            Robot.text("At wall", 5, 75);
            STATE_current = Wall;
            SetState();
          }
        }
      }
    }
    //    delay(1000);
    //    if ((Frontconv < 5) || (Sonconv<5)){
    //    if (Frontconv < 5) {
    //      Robot.motorsWrite(0, 0);
    //      Robot.text("At wall", 5, 75);
    //      GoToAngle(-90);
    //      Robot.motorsWrite(0, 0);
    //      STATE_current = Wall;
    //      SetState();
    //    }

  }
}

//state machine function
void SetState() {
  //if left sonar within range, follow the wall
  if (STATE_current == Wall ) {
    //    STATE_current = Wall;
    updateLCD();
    PD_Left();
  }
  //if rightlight sensor is high, go to follow light
  else if (STATE_current == Light) {
    //    STATE_current = Light;
    updateLCD();
    FollowLight();
  }
  //  //if only right is broken, go to right follow
  else if (STATE_current == PathFrom) {
    //      STATE_current = PathTo;
    updateLCD();
    PathBack();
  }
}

void updateLCD() {
  // clear the LCD screen
  Robot.fill(255, 255, 255);
  Robot.stroke(255, 255, 255);
  Robot.rect(5, 15, 60, 70);
  //printing the current state to the LCD screen
  switch (STATE_current) {
    case Wall:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Wall", 5, 15);
      break;
    case Light:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Light", 5, 15);
      break;
    case PathFrom:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Path From", 5, 15);
      break;
  }
}
//converting function for the front sensor.
float FrontIRConvertToInches(float IRValue) {
  float IRdistance = 0;
  IRdistance = 4122.3 * (pow(IRValue, -1.12));
  return IRdistance;
}
float LeftSonarConvertToInches(float Sonarvalue) {
  float Sonardistance = 0;
  Sonardistance = (0.0069 * Sonarvalue) - 0.6614;
  return Sonardistance;
}
//define robot move funciton for PD control
void RobotMove(float leftOutput, float rightOutput) {
  Robot.motorsWrite(med_spd + leftOutput, med_spd + rightOutput);
  //  delay(250);
  //  Robot.motorsWrite(0, 0);
}
//go to angle function for when front sensor is within range
void GoToAngle(float theta) { //rotates the robot a desired angle
  int turn_delay = 0;
  if (theta > 0) { //Positive theta rotates bot CCW
    Robot.motorsWrite(-fast_spd, fast_spd);
    turn_delay = (theta + 18.833) / 0.3683; //delay based on excel sheet calculation of how far the robot turned at a specified delay

  }
  else if (theta < 0) { //negative theta rotates bot CW
    Robot.motorsWrite(fast_spd, -fast_spd);
    turn_delay = (abs(theta) + 18.833) / 0.3683;
  }
  delay(turn_delay); //delay the movement to reach the desired angle
  Robot.motorsWrite(0, 0);

  delay(250);
}

void ReadSensor(void) {
  //read the values from the sensors
  Leftvalue = Robot.analogRead(LeftPhotoPin);
  Rightvalue = Robot.analogRead(RightPhotoPin);
  Frontvalue = Robot.analogRead(FrontIRPin);

  //  Leftconv = map(Leftvalue, leftMin , leftMax, speedMin, speedMax);
  //  Rightconv = map(Rightvalue, rightMin , rightMax, speedMin, speedMax);
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
  Sonar = pulseIn(LeftSonarPin, HIGH);//measures how long the pin is high
  Sonconv = LeftSonarConvertToInches(Sonar);
  delay(100);
}






