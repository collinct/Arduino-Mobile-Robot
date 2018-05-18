/* Cog-Lab07Part2
 *  ME425-Mobile Robotics
 *  Lab 7 - Metric Path Planning and Execution
 *  
 *  Description:
 *    This code is used to control the robot, plan a path from the robot's starting postion to an ending position,
 *    and then assist the robot in navigating to the ending position. A 4x4 occupancy grid consisting of 0's and 99's
 *    representing free spaces and obstacles is given.  The starting and ending coordinates are also given to the
 *    robot by the user via pushbuttons.  An optimal path is generated using a wavefront expansion algorithm,
 *    which the robot uses to navigate the world map from it's starting position to the ending position.  While
 *    mavigating the world map, the robot uses algorithms from previous labs to follow walls and avoid obstacles.
 *    BUTTON_LEFT -   Decrease the x position of the goal or robot start.
 *    BUTTON_RIGHT -  Increase the x position of the goal or robot start.
 *    BUTTON_UP -     Decrease the y position of the goal or robot start.
 *    BUTTON_DOWN -   Increase the y position of the goal or robot start.
 *    
 *  Primary functions:
 *    SetupPath() -         Takes button inputs to setup the robot start and goal positions.
 *    makeMetricPath() -    Uses wavefront expansion algorithm to plan the path from the robot to the goal. 
 *    NavigateMetricPath -  Navigates the path planned out by the previous function.
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
// converteddistances
float Leftconv = 0;
float Rightconv = 0;
//for converting functions
float IRvalue = 0;
float Sonarvalue = 0;
float Sonardistance = 0;
//IR distances
float Frontconv = 0;
////min and max values for wall
double deadZoneMin = 2;
double deadZoneMax = 20;
// Motor speeds
double med_spd = 150;
double fast_spd = 175;
double speedMin = 100;
double speedMax = 150;
//delays
int movedelay = 750;
int adjustdelay = 500;
//button press boolean
boolean pressed;
//direciton flag to switch between goal and robot
boolean dirflag = false;
int currentWave = 98; // Looking for goal first
// Robot's current position
int robot_x = 0;
int robot_y = 0;
//states for state machine
enum STATE {
  setupPath,
  makePath,
  MetricPath,
  Center,
  Ready
};
//default is setup path
int STATE_current = setupPath;
// Metric MAP
//  0  = free space
//  1  = robot
// 98  = goal
// 99  = obstacle
const int rows = 6;
const int cols = 6;
int MAP[rows][cols] =
{ {99   , 99   , 99   , 99   , 99   , 99},
  {99   , 99   , 0    , 99   , 0    , 99},
  {99   , 99   , 0    , 99   , 0    , 99},
  {99   , 0    , 0    , 0    , 0    , 99},
  {99   , 0   , 99   , 99   , 0    , 99},
  {99   , 99   , 99   , 99   , 99   , 99}
};
int goalX = 0; int goalY = 0;


void setup () {
  // Initialize the Robot, display, and speaker
  Robot.begin();
  Robot.beginTFT();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
  Robot.text("State:", 5, 5);
}

// Loop through reading the sensor and setting the state in the state machine
void loop () {
    ReadSensor();
    SetState();
}
void SetupPath() { //setup the path if the robot by setting the goal and start position
  Robot.text("Xg:", 5, 25); //pring out the coordinates
  Robot.text("Yg:", 5, 45);
  Robot.text("Xs:", 25, 25);
  Robot.text("Ys:", 25, 45);
  while (STATE_current == setupPath) {
    while (dirflag == false) { //while the direction flag is false
      if (Robot.keyboardRead() == BUTTON_MIDDLE) { //if middle button is pressed 
        delay(250);
        Robot.text("start positions", 5, 75); //switch to changing the start position
        MAP[goalX][goalY] = currentWave; //sets the final goal as the currentWave (98)
        dirflag = true; //sets the flag to true so it switches to start position
      }
      else if (Robot.keyboardRead() == BUTTON_RIGHT) {
        goalX++; //increase goal for button right
        Robot.debugPrint(goalX, 5, 35); //print the goalX
      }
      else if (Robot.keyboardRead() == BUTTON_LEFT) {
        goalX--; //decrease goal for button left
        Robot.debugPrint(goalX, 5, 35); //print the goalX
      }
      else if (Robot.keyboardRead() == BUTTON_UP) {
        goalY++; //increase goal for button up
        Robot.debugPrint(goalY, 5, 55); //print the goalY
      }
      else if (Robot.keyboardRead() == BUTTON_DOWN) {
        goalY--; //decrease goal for button down
        Robot.debugPrint(goalY, 5, 55); //print the goalY
      }
    }
    while (dirflag == true) { //while flag is true
      if (Robot.keyboardRead() == BUTTON_MIDDLE) { //if middle button pressed
        MAP[robot_x][robot_y] = 1; //set the current robot x and y to 1 (start point)
        STATE_current = makePath; //change state to make path
        SetState(); 
        break;
        break; //break out of the while loops
      }
      else if (Robot.keyboardRead() == BUTTON_RIGHT) {
        robot_x++; //increase start for button right
        Robot.debugPrint(robot_x, 25, 35); //print the goalX
      }
      else if (Robot.keyboardRead() == BUTTON_LEFT) {
        robot_x--; //decrease start for button left
        Robot.debugPrint(robot_x, 25, 35); //print the goalX
      }
      else if (Robot.keyboardRead() == BUTTON_UP) {
        robot_y++; //increase start for button up
        Robot.debugPrint(robot_y, 25, 55); //print the goalX
      }
      else if (Robot.keyboardRead() == BUTTON_DOWN) {
        robot_y--; //decrease start for button down
        Robot.debugPrint(robot_y, 25, 55); //print the goalX
      }
    }
  }
}
// Wavefront algorithm to find most efficient path to goal
void makeMetricPath () {
  while (STATE_current == makePath) { //while state is makePath
    bool foundWave = true; //set found wave to true
    if (Robot.keyboardRead() == BUTTON_MIDDLE) { //if middle button is pressed
      STATE_current = MetricPath; //move on to executing the path
      SetState(); //set state
      break;
    }
    while (foundWave == true) { //while the flag is true
      foundWave = false; //set flag to false
      for (int y = 0; y < cols; y++) {
        for (int x = 0; x < rows; x++) {
          if (MAP[x][y] == currentWave) { //search through the grid and if it is the goal
            foundWave = true; //set the flag back to true
            goalX = x; //set goalX to the current position
            goalY = y; //set goalY to current position

            if (goalX > 0) { // Checks the array bounds to the West
              if (MAP[goalX - 1][goalY] == 0)  { // Checks West direction
                MAP[goalX - 1][goalY] = currentWave - 1; //makes the west cell one less than the previous number
              }
            }

            if (goalX < (rows - 1)) { // Checks the array bounds to the East
              if (MAP[goalX + 1][goalY] == 0) { // Checks East direction
                MAP[goalX + 1][goalY] = currentWave - 1; //makes east cell one less than the  previous number
              }
            }

            if (goalY > 0) { // Checks the array bounds to the South
              if (MAP[goalX][goalY - 1] == 0) { // Checks South direction
                MAP[goalX][goalY - 1] = currentWave - 1; //makes south cell one less than the  previous number
              }
            }

            if (goalY < (cols - 1)) { // Checks the array bounds to the North
              if (MAP[goalX][goalY + 1] == 0) { // Checks North direction
                MAP[goalX][goalY + 1] = currentWave - 1; //makes north cell one less than the  previous number
              }
            }
          }
        }
      }
      currentWave--; //decrease the current wave by 1
      printMAP(); //print the map on the LCD
    }
  }
}

//  0  = free space
//  1  = robot
// 98  = goal
// 99  = obstacle

//0  = open space
//1  = barrier
//2  = goal
//99 = robot

// Navigate most efficient path to goal and update screen map as robot moves
void NavigateMetricPath() {
  //  while (STATE_current == MetricPath) {
  // Find robot coordinates
  for (int x = 0; x < rows; x++) {
    for (int y = 0; y < cols; y++) {
      if (MAP[x][y] == 1) { //finds the robot coordinates
        robot_x = x; //sets the robot_x to that x value
        robot_y = y; //sets the robot_y to that y value
      }
    }
  }
  // Decide path
  //initialize
  int current_x = robot_x; //sets current x location to the robot spot
  int current_y = robot_y; //sets current y location to the robot spot
  int current_facing = 1; //current facing is south
  int next_Direction = 1; //next direction is couth
  int current_high = 1; //initialize where 

  while (current_high < 98) { //while location is not the goal
    current_high = 1; // Every time, reset to lowest number (robot == 1)
    next_Direction = current_facing; //next direction is the current direciton it is facing
    int Next_X = 0;
    int Next_Y = 0;

    if (current_x > 0) { // Checks the array bounds to the West
      if ((MAP[current_x - 1][current_y] > current_high) && (MAP[current_x - 1][current_y] != 99)) { // Checks if current space is occupied
        current_high = MAP[current_x - 1][current_y]; // Sets next number
        next_Direction = 3; // Sets next direction as West
        Next_X = current_x - 1; //next x value is to the west
        Next_Y = current_y; //same y vlaue
      }
    }

    if (current_x < (rows - 1)) { // Checks the array bounds to the East
      if ((MAP[current_x + 1][current_y] > current_high) && (MAP[current_x + 1][current_y] != 99)) { // Checks if current space is occupied
        current_high = MAP[current_x + 1][current_y]; // Sets next number
        next_Direction = 1; // Sets next direction as East
        Next_X = current_x + 1; //next x value is to the east
        Next_Y = current_y; //same y value
      }
    }

    if (current_y > 0) { // Checks the array bounds to the South
      if ((MAP[current_x][current_y - 1] > current_high) && (MAP[current_x][current_y - 1] != 99)) { // Checks if current space is occupied
        current_high = MAP[current_x][current_y - 1]; // Sets next number
        next_Direction = 2; // Sets next direction as South
        Next_X = current_x; //same x value
        Next_Y = current_y - 1; //next y value is to the south
      }
    }

    if (current_y < (cols - 1)) { // Checks the array bounds to the North
      if ((MAP[current_x][current_y + 1] > current_high) && (MAP[current_x][current_y + 1] != 99)) { // Checks if current space is occupied
        current_high = MAP[current_x][current_y + 1]; // Sets next number
        next_Direction = 0; // Sets next direction as North
        Next_X = current_x; //same x value
        Next_Y = current_y + 1; //next y value is to the north
      }
    }

    current_x = Next_X; 
    current_y = Next_Y;
    MAP[current_x][current_y] = 2; //where the robot is. changes number so it is obvious on the LCD
    //Track the robot's heading
    while (current_facing != next_Direction) { //while the current direction is not the next direction
      if (current_facing > next_Direction) { //if current facing is right of the next direction
        GoToAngle(100); //turn left 90 degrees
        Robot.text("Turn Left", 5, 95); //print to LCD
        current_facing--; //decrease current facing
      }
      else if (current_facing < next_Direction) { //else if current facing is left of the next direction
        GoToAngle(-100); //turn right 90 degrees
        Robot.text("Turn Right", 5, 105); //print to LCD
        current_facing++; //decrease current facing
      }
    }
    Robot.motorsWrite(fast_spd, fast_spd); //move robot forward
    Robot.text("Forward", 5, 115);
    delay(movedelay); //delay for a time that the robot moves about 14 inches
    CenterFollow(); //go to center follow to adjust robot if need be
    Robot.motorsWrite(0, 0);
    delay(500); //wait a bit
    if (current_high == 98) { //if the robot is at the goal 
      Robot.motorsWrite(0, 0); //stop the robot
      STATE_current = Ready; //go to ready state
      SetState();
      break;
    }
    printMAP(); //print map to LCD
    updateLCD(); //update the LCD with new values
  }
  //  }
}
void CenterFollow() { //center follow function
  int finish = millis() + adjustdelay; //finish time is the current time plus the delay of the robot moving
  while (millis() < finish) { //while the current time is less than the finish time
    Robot.text("adjusting", 5, 125); //print that the robot is adjusting
    ReadSensor(); //read the sensors
    if (Leftconv < 3) { //if the left distance is really close
      Rightvalue = speedMax; //speed up the right wheel
      Leftvalue = speedMin; //slow down the left wheel
    }
    else if (Rightconv < 3) { //if right distance is really close
      Leftvalue = speedMax; //speed up the left wheel
      Rightvalue = speedMin; //slow down the right wheel
    }
    else { //otherwise adjust the wheels 
      Leftvalue = map(Leftconv, deadZoneMin, deadZoneMax, speedMin, speedMax);
      Rightvalue = map(Rightconv, deadZoneMin, deadZoneMax, speedMin, speedMax);
    }
    Robot.motorsWrite(Rightvalue, Leftvalue); //move the robot
  }


}
void printMAP() { //prints the map on the LCD
  int xx=5;
  int yy=15;
  for (int x = 0; x < rows; x++) { //Scroll down the MAP Array
    for (int y = 0; y < cols; y++) { //Scroll across the MAP Array
      Robot.debugPrint(MAP[x][y], xx+5, yy+15);
      xx= xx+15; //increase the next x value 15 pixels
    }
    yy=yy+10; //increase the next y value by 10
    xx=5; //set next x value to 5 pixels from the wall
  }
}

//state machine function
void SetState(void) {
  switch (STATE_current) {
    case setupPath: //
      updateLCD(); //update the LCD
      SetupPath();
    case makePath: //
      updateLCD(); //update the LCD
      makeMetricPath();
    case MetricPath: //
      updateLCD(); //update the LCD
      NavigateMetricPath();
    case Ready: //
      updateLCD(); //update the LCD
      ReadyState();
    case Center: //
      updateLCD(); //update the LCD
      CenterFollow();
  }
}

void updateLCD() {
  // clear the LCD screen
  Robot.fill(255, 255, 255);
  Robot.stroke(255, 255, 255);
  Robot.rect(5, 15, 100, 135);
  //printing the current state to the LCD screen
  switch (STATE_current) {
    case setupPath:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Setup Path", 5, 15);
      break;
    case makePath:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Plan Path", 5, 15);
      break;
    case MetricPath:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Metric Nav Exec", 5, 15);
      break;
    case Ready:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Ready", 5, 15);
      break;
    case Center:
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
void ReadyState() {
  while (STATE_current == Ready) {
    //do nothing except read buttons
    Robot.motorsWrite(0, 0);
    Robot.text("At goal", 5, 55);
  }
}


