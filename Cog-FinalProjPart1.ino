/* Cog-FinalProjPart1
 *  ME425-Mobile Robotics
 *  Final Project Part 1 - Mapping
 *  
 *  Description:
 *    This code is used to control the robot to move about a 4x4 matrix and map the cells in the matrix as either
 *    open or closed.  After mapping the 4x4 matrix, the robot plans a path back to it's starting position and
 *    navigates the planned path.
 *    BUTTON_LEFT -  Decrease the x position of the goal or robot start.
 *    BUTTON_RIGHT - Increase the x position of the goal or robot start.
 *    BUTTON_UP -    Decrease the y position of the goal or robot start.
 *    BUTTON_DOWN -  Increase the y position of the goal or robot start.
 *  
 *  Primary functions:
 *    ConvertingMap() -    Converts the map from 1-15s to 0s and 99s.
 *    SetupPath() -        Takes button inputs to setup the robot start and goal positions.
 *    makeMetricPath() -   Uses wavefront algorithm to plan the path from the robot to the goal.
 *    NavigateMetricPath - Navigates the path planned out by the previous function.
 *  
 *  Secondary functions:
 *    SetState() -                                      State machine for setting the state between layers.
 *    updateLCD() -                                     Updates the LCD based on the state.
 *    Center() -                                        Wall following to help keep the robot in the center of the hallways.
 *    ReadyState() -                                    Filler layer for the robot to go to when middle button
 *                                                        is pressed. stops the robot.
 *    FrontIRConvertToInches(float IRValue) -           Converting function for the front IR sensor.
 *    LeftSonarConvertToInches(float Sonarvalue) -      Converting function for the left sonar sensor.
 *    RightSonarConvertToInches(float Sonarvalue) -     Converting function for the left sonar sensor.
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
#include <Math.h>

// Sensor pins
#define LeftSonarPin TKD2
#define RightSonarPin TKD0
#define FrontIRPin TK2

// Sensor values
float Sonar = 0;
int Frontvalue = 0;
int Leftvalue = 0;
int Rightvalue = 0;
// Converteddistances
float Leftconv = 0;
float Rightconv = 0;
//for converting functions
float IRvalue = 0;
float Sonarvalue = 0;
float Sonardistance = 0;
//IR distances
float Frontconv = 0;
// Motor speeds
double med_spd = 150;
double fast_spd = 175;
int movedelay = 1250;
//// Robot's current position
//int robot_x = 0;
//int robot_y = 0;
//states for state machine
enum STATE {
  Reading,
  Return,
  Ready
};

int STATE_current = Reading;
// Metric MAP
//  0  = free space
//  1  = robot
// 98  = goal
// 99  = obstacle
const int rows = 6;
const int cols = 6;
int MAP[rows][cols] =
{ {99   , 99   , 99   , 99  , 99  , 99},
  {99   , 2   ,  99   ,  99   , 20  , 99},
  {99   , 2   ,  99   ,  99  , 2   , 99},
  {99   , 2   ,  2   ,  2   , 2   , 99},
  {99   , 0   ,  99   ,  99   , 2   , 99},
  {99   , 99  , 99   , 99   , 99  , 99}
};
//{ {99   , 99   , 99   , 99  , 99  , 99},
//  {99   , 0   ,  0   ,  0   , 20  , 99},
//  {99   , 0   ,  0   ,  0   , 0   , 99},
//  {99   , 0   ,  0   ,  0   , 0   , 99},
//  {99   , 0   ,  0   ,  0   , 0   , 99},
//  {99   , 99  , 99   , 99   , 99  , 99}
//};



void setup () {
  // Initialize the Robot, display, and speaker
  Robot.begin();
  Robot.beginTFT();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
  Robot.text("State", 5, 5);

}

// Loop through reading the sensor and setting the state in the state machine
void loop () {
  ReadSensor();
  SetState();
}
void MakeMap() { //converting the map from 1-15s to 0s and 99s
  // Robot's current position
  char robot_x = 0;
  char robot_y = 0;
  // Find robot coordinates
  for (int x = 0; x < rows; x++) {
    for (int y = 0; y < cols; y++) {
      if (MAP[x][y] == 20) {
        robot_x = x;
        robot_y = y;
      }
    }
 }
  //initialize
  char current_facing = 1; //current facing is east
  char next_Direction = 1; //next direction is east
  //  int current_facing = 0; //current facing is north
  //  int next_Direction = 0; //next direction is north
  //  int current_x = robot_x; //sets current x location to the robot spot
  //  int current_y = robot_y; //sets current y location to the robot spot
  char current_x = 3; //sets current x location to the robot spot
  char current_y = 1; //sets current y location to the robot spot

  printMAP(); //print the map to the LCD
  while (STATE_current == Reading) { //while it is in the convertMap state
    boolean doneflag = true;

    char Next_X = 0;
    char Next_Y = 0;
   next_Direction = current_facing;
    ReadSensor();
    //exit if all cells mapped
    for (int x = 0; x < rows; x++) {
      for (int y = 0; y < cols; y++) {
        //        if ((MAP[x][y] == 0) || (MAP[x][y] == 99) || (MAP[x][y] == 20) || (MAP[x][y] == 2)) {
        if (MAP[x][y] == 0) {
          doneflag = false;
        }
      }//end for
    }//end for
    if (doneflag == true) {
      //      MAP[current_x][current_y]=30;
      Robot.motorsWrite(0, 0);
      STATE_current = Return;
      SetState();
    }
    if (doneflag == false) {
      if (current_facing == 1) { // if it is facing East
        if (Frontconv < 7) {
          MAP[current_x + 1][current_y] = 99; // Sets next number
          //        next_Direction = 3; // Sets next direction as West
          Next_X = current_x; //Same x
          Next_Y = current_y - 1; //next y value is to the South
        }
        if (Leftconv < 7) {
          MAP[current_x][current_y + 1] = 99; // Sets next number
          //        next_Direction = 3; // Sets next direction as West
          Next_X = current_x + 1; //next x value is to the east
          Next_Y = current_y; //same y vlaue
        }
        if (Rightconv < 7) {
          MAP[current_x][current_y - 1] = 99; // Sets next number
          //        next_Direction = 3; // Sets next direction as West
          Next_X = current_x + 1; //next x value is to the west
          Next_Y = current_y; //same y vlaue
        }
        if (((Rightconv < 7) && (Frontconv < 7)) && (Leftconv > 7)) { //right corner
          next_Direction = 0;
          Next_X = current_x; //same x
          Next_Y = current_y + 1; //next y to North
        }
        if (((Leftconv < 7) && (Frontconv < 7)) && (Rightconv > 7)) { //left corner
          next_Direction = 2;
          Next_X = current_x; //same x
          Next_Y = current_y - 1; //next y to South
        }
        if ((Leftconv < 7) && (Frontconv < 7) && (Rightconv < 7)) { //U in front
          next_Direction = 3; //180 degrees around
          Next_X = current_x - 1; //next x to west
          Next_Y = current_y ; //same y
        }
      }

      if (current_facing == 2) { // if it is facing South
        if (Frontconv < 7) {
          MAP[current_x ][current_y - 1] = 99; // Sets next number
          next_Direction = 3; // Sets next direction as West
          Next_X = current_x - 1; //next x to west
          Next_Y = current_y ; //same y
        }
        if (Leftconv < 7) {
          MAP[current_x + 1][current_y ] = 99; // Sets next number
          //        next_Direction = 3; // Sets next direction as West
          Next_X = current_x; //same x
          Next_Y = current_y - 1; //next y to south
        }
        if (Rightconv < 7) {
          MAP[current_x - 1][current_y] = 99; // Sets next number
          //        next_Direction = 3; // Sets next direction as West
          Next_X = current_x; //same x
          Next_Y = current_y - 1; //next y to south
        }
        if (((Rightconv < 7) && (Frontconv < 7)) && (Leftconv > 7)) { //right corner
          next_Direction = 1;
          Next_X = current_x + 1; //next x to east
          Next_Y = current_y ; //same y
        }
        if (((Leftconv < 7) && (Frontconv < 7)) && (Rightconv > 7)) { //left corner
          next_Direction = 3;
          Next_X = current_x - 1; //next y to west
          Next_Y = current_y; //same y
        }
        if ((Leftconv < 7) && (Frontconv < 7) && (Rightconv < 7)) { //U in front
          next_Direction = 0; //180 degrees around
          Next_X = current_x; //same x
          Next_Y = current_y + 1 ; //next y to north
        }
      }

      if (current_facing == 3) { // if it is facing west
        if (Frontconv < 7) {
          MAP[current_x - 1][current_y] = 99; // Sets next number
          //        next_Direction = 3; // Sets next direction as West
          Next_X = current_x; //Same x
          Next_Y = current_y - 1; //next y value is to the south
        }
        if (Leftconv < 7) {
          MAP[current_x][current_y - 1] = 99; // Sets next number
          //        next_Direction = 3; // Sets next direction as West
          Next_X = current_x - 1; //next x value is to the west
          Next_Y = current_y; //same y vlaue
        }
        if ((Rightconv < 7) && (Leftconv > 8)) {
          MAP[current_x][current_y + 1] = 99; // Sets next number
          next_Direction = 2; // Sets next direction as south
          Next_X = current_x; // same x
          Next_Y = current_y - 1; //next y to south
        }
        if (((Rightconv < 7) && (Frontconv < 7)) && (Leftconv > 7)) { //right corner
          next_Direction = 2;
          Next_X = current_x; //same x
          Next_Y = current_y - 1; //next y to south
        }
        if (((Leftconv < 7) && (Frontconv < 7)) && (Rightconv > 7)) { //left corner
          next_Direction = 0;
          Next_X = current_x; //same x
          Next_Y = current_y + 1; //next y to north
        }
        if ((Leftconv < 7) && (Frontconv < 7) && (Rightconv < 7)) { //U in front
          MAP[current_x][current_y + 1] = 99; // Sets next number
          MAP[current_x - 1][current_y ] = 99; // Sets next number
          MAP[current_x][current_y - 1] = 99; // Sets next number
          next_Direction = 1; //180 degrees around
          Next_X = current_x + 1; //next x to east
          Next_Y = current_y ; //same y
        }
      }

      if (current_facing == 0) { // if it is facing North
        if (Frontconv < 7) {
          MAP[current_x ][current_y + 1] = 99; // Sets next number
          //        next_Direction = 3; // Sets next direction as West
          Next_X = current_x + 1; //next x to east
          Next_Y = current_y ; //same y
        }
        if (Leftconv < 7) {
          MAP[current_x - 1][current_y ] = 99; // Sets next number
          //        next_Direction = 3; // Sets next direction as West
          Next_X = current_x; //same x
          Next_Y = current_y + 1; //next y to north
        }
        if (Rightconv < 7) {
          MAP[current_x + 1][current_y] = 99; // Sets next number
          //        next_Direction = 3; // Sets next direction as West
          Next_X = current_x; //same x
          Next_Y = current_y + 1; //next y to north
        }
        if (((Rightconv < 7) && (Frontconv < 7)) && (Leftconv > 7)) { //right corner
          next_Direction = 3;
          Next_X = current_x - 1; //next x to west
          Next_Y = current_y ; //same y
        }
        if (((Leftconv < 7) && (Frontconv < 7)) && (Rightconv > 7)) { //left corner
          next_Direction = 1;
          Next_X = current_x + 1; //next y to east
          Next_Y = current_y; //same y
        }
        if ((Leftconv < 7) && (Frontconv < 7) && (Rightconv < 7)) { //U in front
          next_Direction = 2; //180 degrees around
          Next_X = current_x - 1; //next x to west
          Next_Y = current_y ; //same y
        }
      }
      current_x = Next_X; //sets the current_x
      current_y = Next_Y; //sets the current_y
      MAP[current_x][current_y] = 2; //where the robot is make it a 2

      //Track the robot's heading
      while (current_facing != next_Direction) { //while the current direction is not the next direction
        if (current_facing > next_Direction) { //if current facing is right of the next direction
          GoToAngle(100); //turn left 90 degrees
          Robot.text("Left", 5, 95); //print to LCD
          current_facing--; //decrease current facing
        }
        else if (current_facing < next_Direction) { //else if current facing is left of the next direction
          GoToAngle(-100); //turn right 90 degrees
          Robot.text("Right", 5, 105); //print to LCD
          current_facing++; //decrease current facing
        }
      }
      Robot.motorsWrite(fast_spd, fast_spd); //move robot forward
      Robot.text("Forward", 5, 115);
      delay(movedelay); //delay for a time that the robot moves about 18 inches
      //    LeftWall(); //go to center follow to adjust the robot if need be
      Robot.motorsWrite(0, 0); //stop the robot
      delay(500); //wait 0.5 sec
      printMAP(); //print map
      delay(500); //wait 0.5 sec
      updateLCD(); //update the LCD
    }
  }
}
void ReturnPath() {
  //initialize
  GoToAngle(220);
  delay(500);
  char current_facing = 3; //current facing is east
  char next_Direction; //next direction is west
  //  int current_facing = 0; //current facing is north
  //  int next_Direction = 0; //next direction is north
  char current_x = 4; //sets current x location to the robot spot
  char current_y = 1; //sets current y location to the robot spot

  //  printMAP(); //print the map to the LCD
  while (STATE_current == Return) { //while it is in the convertMap state
    //    boolean doneflag = true;
    int Next_X = 0;
    int Next_Y = 0;
    next_Direction = current_facing;
    ReadSensor();
    if (current_facing == 3) { // if it is facing west
      if ((Rightconv < 7) && (Leftconv < 7)) {
        Next_X = current_x - 1; //next x value is to the west
        Next_Y = current_y; //same y vlaue
      }
      if ((Rightconv > 7) && (Leftconv < 7)) {
        next_Direction = 0; // Sets next direction as north
        Next_X = current_x; // same x
        Next_Y = current_y + 1; //next y to north
      }
    }
    if (current_facing == 0) { // if it is facing North
      if (Frontconv < 7) {
        next_Direction = 3; // Sets next direction as West
        Next_X = current_x - 1; //next x to west
        Next_Y = current_y ; //same y
      }
      if (Leftconv < 7) {
        Next_X = current_x; //same x
        Next_Y = current_y + 1; //next y to north
      }
    }
    current_x = Next_X; //sets the current_x
    current_y = Next_Y; //sets the current_y
    MAP[current_x][current_y] = 5; //where the robot is make it a 5

    //Track the robot's heading
    while (current_facing != next_Direction) { //while the current direction is not the next direction
      if (current_facing > next_Direction) { //if current facing is right of the next direction
        GoToAngle(100); //turn left 90 degrees
        //          Robot.text("Left", 5, 95); //print to LCD
        current_facing--; //decrease current facing
      }
      else if (current_facing < next_Direction) { //else if current facing is left of the next direction
        GoToAngle(-100); //turn right 90 degrees
        //          Robot.text("Right", 5, 105); //print to LCD
        current_facing++; //decrease current facing
      }
    }
    Robot.motorsWrite(fast_spd, fast_spd); //move robot forward
    Robot.text("Go", 5, 115);
    delay(movedelay); //delay for a time that the robot moves about 18 inches
    //    LeftWall(); //go to center follow to adjust the robot if need be
    Robot.motorsWrite(0, 0); //stop the robot
    delay(500); //wait 0.5 sec
    if (MAP[current_x -1][current_y] == 20) {
      Robot.motorsWrite(fast_spd, fast_spd); //move robot forward
      delay(movedelay); //delay for a time that the robot moves about 18 inches
      Robot.motorsWrite(0, 0); //stop the robot
//        doneflag == true;
        STATE_current=Ready;
        SetState();
      break;
    }
    printMAP(); //print map
    delay(500); //wait 0.5 sec
    updateLCD(); //update the LCD
  }
}
void printMAP() { //prints the map on the LCD
  int xx = 5;
  int yy = 15;
  for (int x = 0; x < rows; x++) { //Scroll down the MAP Array
    for (int y = 0; y < cols; y++) { //Scroll across the MAP Array
      //      Serial.print(MAP[x][y]);
      //      Serial.print(" ");
      Robot.debugPrint(MAP[x][y], xx + 5, yy + 15);
      xx = xx + 15; //increase the next x value 15 pixels
    }
    //    Serial.println();
    yy = yy + 10; //increase the next y value by 10
    xx = 5; //set next x value to 5 pixels from the wall
  }
}

//state machine function
void SetState(void) {
  switch (STATE_current) {
    case Reading: //
      updateLCD(); //update the LCD
      MakeMap();
    case Return: //
      updateLCD(); //update the LCD
      ReturnPath();
    case Ready: //
      updateLCD(); //update the LCD
      ReadyState();
  }
}

void updateLCD() {
  // clear the LCD screen
  Robot.fill(255, 255, 255);
  Robot.stroke(255, 255, 255);
  Robot.rect(5, 15, 100, 135);
  //printing the current state to the LCD screen
  switch (STATE_current) {
    case Reading:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Map", 5, 15);
      break;
    case Return:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Return", 5, 15);
      break;
    case Ready:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Ready", 5, 15);
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
  //convert IR readings to inches via the called function
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
void ReadyState() {
  while (STATE_current == Ready) { //while in ready state.
    //do nothing except print to LCD
    printMAP();
    Robot.motorsWrite(0, 0);
    Robot.text("At goal", 5, 125);
  }
}


