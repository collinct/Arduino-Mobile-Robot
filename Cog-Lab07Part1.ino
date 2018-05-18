/* Cog-Lab07Part1
 *  ME425-Mobile Robotics
 *  Lab 7 Part 1 - Topological Navigation
 *  
 *  Description:
 *    This code is used to control the robot to move about a 4x4 matrix world by using the robot's sensors to identify
 *    various gateways and places and then navigate accordingly.  Initally, the robot is given various commands
 *    by the user in the form of an array of values via push buttons to assist in path execution.
 *    BUTTON_LEFT -   Store a 2 in the array and go left when it comes to an opening.
 *    BUTTON_RIGHT -  Store a 1 in the array and go right when it comes to an opening.
 *    BUTTON_UP -     Store a 3 in the array and go straight to bypass the opening.
 *    
 *  Primary functions:
 *    TopoSetup() - Takes button inputs to store array.
 *    TopoExec() -  Reads the array and executes the respective movement.
 *    
 *  Secondary functions:
 *    SetState() -                                      State machine for setting the state between layers.
 *    updateLCD() -                                     Updates the LCD based on the state.
 *    Center() -                                        Wall following to help keep the robot in the center of the hallways.
 *    ReadyState() -                                    Filler layer for the robot to go to when middle button
 *                                                        is pressed. stops the robot.
 *    FrontIRConvertToInches(float IRValue) -           Converting function for the front IR sensor.
 *    LeftSonarConvertToInches(float Sonarvalue) -      Converting function for the left sonar sensor.
 *    RobotMove(float leftOutput, float rightOutput) -  Moving function for the wall following.
 *    GoToAngle(float theta) -                          Turns the robot a desired angle.
 *    ReadSensor() -                                    Reads all sensors.
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
#define LeftSonarPin TKD2
#define RightSonarPin TKD0
#define FrontIRPin TK2

// Sensor values
float Sonar = 0;
int Frontvalue = 0;
int Leftvalue = 0;
int Rightvalue = 0;
// Converted distances
float Leftconv = 0;
float Rightconv = 0;
//for converting functions
float IRvalue = 0;
float Sonarvalue = 0;
float Sonardistance = 0;
//IR distances
float Frontconv = 0;
// Motor speeds
double med_spd  = 115;
double fast_spd = 175;
// Min and max values for wall following
double deadZoneMin = 4;
double deadZoneMax = 6;
// Center Variables
float errorLeft = 0;
float errorRight = 0;
float derivativeLeft = 0;
float derivativeRight = 0;
float leftOutput = 0;
float rightOutput = 0;
float leftErrorPrior = 0;
float rightErrorPrior = 0;
int time1 = 0;
// Desired sensor value
float sensorVal_des = 5;
// PD constants
double Kp = 5;
float Kd = 0.005;

// Array aetup
#define pathlength 5
int path[pathlength];
int index = 0;
//button press
boolean pressed;
//boolean for TopoSetup
boolean done = false;
//enum for the state selection
enum STATE {
  Ready,
  TopoNavSetup,
  TopoNavExec,
  CenterFollow,
};
//default state is the Ready state
int STATE_current = Ready;
void setup() {
  // initialize the Robot and display
  Robot.begin();
  Robot.beginTFT();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
  Robot.text("State:", 5, 5);
  //makes all values of array 0s
  for (int i = 0; i < pathlength; i++) {
    path[i] = 0;
  }
}
void loop() {
  //read the sensors
  ReadSensor();
  //run the state machine if a button is pressed
  pressed = SetState(Robot.keyboardRead());
  //update LCD if button pressed
  if (pressed) {
    updateLCD();
  }
}

//derivative control for left wall follow
void TopoSetup() {
  delay(1000);
  while (done == false) {
    for (int i = 0 ; i < pathlength - 1; i++) {
      if (Robot.keyboardRead() == BUTTON_LEFT) { //if left button is pressed
        path[index] = 2; //go left
        delay(250);
        //print out the path
        Robot.debugPrint(path[0], 5, 25);
        Robot.debugPrint(path[1], 5, 35);
        Robot.debugPrint(path[2], 5, 45);
        index++;
      }
      else if (Robot.keyboardRead() == BUTTON_RIGHT) { //if right button is pressed
        path[index] = 1; //go right
        delay(250);
        //print out the path
        Robot.debugPrint(path[0], 5, 25);
        Robot.debugPrint(path[1], 5, 35);
        Robot.debugPrint(path[2], 5, 45);
        index++;
      }
      else if (Robot.keyboardRead() == BUTTON_UP) { //if up button is pressed
        path[index] = 3; //forward
        delay(250);
        //print out the path
        Robot.debugPrint(path[0], 5, 25);
        Robot.debugPrint(path[1], 5, 35);
        Robot.debugPrint(path[2], 5, 45);
        index++;
      }
      else if (Robot.keyboardRead() == BUTTON_DOWN) { //stop the robot when 4 is reached. move on to the center follow and path execution
        path[index] = 4; //stop
        delay(250);
        Robot.debugPrint(path[index], 5, 25); //print the index value on the screen
        done = true;
        index = 0;
        STATE_current = CenterFollow; //change the state and go to the state machine
        updateLCD();
        Center();
        break;
      }
    }

    if (Robot.keyboardRead() == BUTTON_MIDDLE) {
      Robot.motorsWrite(0, 0);
      delay(100);
      SetState(Robot.keyboardRead()); //go to the ready state if you press middle button
      break;
    }
  }
}

void TopoExec() {
  
  while (STATE_current == TopoNavExec) { //while the robot is in the TopoNavExec state 
    ReadSensor(); //read sensor
    if (path[index] == 3) { //if array value is 3 
      Robot.motorsWrite(med_spd, med_spd); //move forward 
      delay(1000); //delay for a bit to move past the opening
      Robot.motorsWrite(0, 0);
      delay(250);
      index++; //increase the index

      STATE_current = CenterFollow; //go back to the center follow function
      updateLCD();
      Center();
      break;
    }
    else if (path[index] == 1) { //if array value was a 1
      delay(250);
      GoToAngle(-100); //rotate right 90 degrees (give or take a few degrees)
      Robot.motorsWrite(fast_spd, fast_spd);    //go forward to get out of that cell
      delay(900);
      Robot.motorsWrite(0, 0);
      delay(250);
      index++; //increase index
      STATE_current = CenterFollow; //go back to the center follow to follow the wall and look for next index
      updateLCD();
      Center();
      break;
    }
    else if (path[index] == 2) { //if the next vlaue in the array is a 2
      delay(250);
      GoToAngle(90); //turn left 90 degrees
      Robot.motorsWrite(fast_spd, fast_spd); //go forward to next cell
      delay(1000);
      Robot.motorsWrite(0, 0);
      delay(250);
      index++; //increase index
      STATE_current = CenterFollow; //go back to the center follow to follow the wall and look for next index
      updateLCD();
      Center();
      break;
    }
    if ((path[index] == 4) && (Frontconv < 5)) { //if the index is the last one and the front sensor is close
      delay(250);
      Robot.motorsWrite(0, 0); //stop the robot
      delay(250);
      STATE_current = Ready; //go to ready state
      updateLCD();
      ReadyState();
    }
    //      }

    if (Robot.keyboardRead() == BUTTON_MIDDLE) { //go to ready state if middle button is pressed (safety measure)
      Robot.motorsWrite(0, 0);
      delay(100);
      SetState(Robot.keyboardRead());
      break;
    }
    //    }
  }
}
void Center() {
  while (STATE_current == CenterFollow) { //while the robot is in CenterFollow state
    ReadSensor(); //read the sensors
    RobotMove(leftOutput, rightOutput); //call robot move function
    errorRight = 0; //initialize the errors
    errorLeft = 0;
    errorRight = sensorVal_des - Rightconv; //compare the actual distance with the desired value (5)
    errorLeft = sensorVal_des - Leftconv;
    if ((path[index] == 4) && (Frontconv < 5)) { //if the index is the last one and the front sensor is close
      delay(250);
      Robot.motorsWrite(0, 0); //stop the robot
      delay(250);
      STATE_current = Ready; //go to ready state
      updateLCD();
      ReadyState();
    }
    //if robot is in dead (4-6 in)
    if ((abs(errorRight) <= (deadZoneMax - sensorVal_des)) && (abs(errorLeft) <= (deadZoneMax - sensorVal_des))) {
      ReadSensor();
      rightOutput = 0; //dont adjust speed if its in the desired zone
      leftOutput = 0;
    }
    //if either of the sides go beyond range, go to state machine
    else if ((Rightconv > 13) || (Leftconv > 13)) {
      delay(1000);
      STATE_current = TopoNavExec; //set state to TopoNavExec
      Robot.motorsWrite(0, 0); //stop the robot
      delay(250);
      updateLCD();
      TopoExec(); //go to the TopoExec function
      break;
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
void ReadyState() {
  while (STATE_current == Ready) {
    //do nothing except read buttons
    Robot.keyboardRead();
  }
}
//state machine function
boolean SetState(int keyCode) {
  switch (keyCode) {
    case BUTTON_MIDDLE: //
      STATE_current = Ready; //if middle button pressed go to ready state
      updateLCD(); //update the LCD
      ReadyState();
      return true;
    case BUTTON_UP: //
      STATE_current = TopoNavSetup; //if up button is pressed go to agression state
      updateLCD(); //update the LCD
      TopoSetup();
      return true;
  }
  return false; // if no buttons pressed
}

void updateLCD() {
  // clear the LCD screen
  Robot.fill(255, 255, 255);
  Robot.stroke(255, 255, 255);
  Robot.rect(5, 15, 90, 50);
  //printing the current state to the LCD screen
  switch (STATE_current) {
    case Ready:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Ready", 5, 15);
      break;
    case TopoNavSetup:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Topo Nav Setup", 5, 15);
      break;
    case TopoNavExec:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Topo Nav Exec", 5, 15);
      break;
    case CenterFollow:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Center", 5, 15);
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
float RightSonarConvertToInches(float Sonarvalue) {
  Sonardistance = (0.0068 * Sonarvalue) - 0.7455;
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






