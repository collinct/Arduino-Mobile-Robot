/* Cog-FinalProjPart2
 *  ME425-Mobile Robotics
 *  Final Project Part 2 - Localization
 *  
 *  Description:
 *    This code is used to control the robot to move about a 4x4 matrix grid and localize it's position in said grid.
 *    A completed 4x4 grid is given to the robot, whose location is initially unknown.  The robot is placed in the
 *    grid and localizes itself by:
 *    1.) sensing landmarks in its surroundings,
 *    2.) comparing the sensed landmarks to the stored landmarks in the given 4x4 matrix,
 *    3.) determining all possible locations in the grid that are a match and eliminating
 *          the locations that aren't a match,
 *    4.) navigating to another cell in the grid, and
 *    5.) repeating the aforementioned process until only one viable location remains,
 *          successfully localizing the robot.
 *    After successful localization, the robot plans a path to a specified location in the grid and navigates to it.
 *    BUTTON_LEFT -  Decrease the x position of the goal or robot start.
 *    BUTTON_RIGHT - Increase the x position of the goal or robot start.
 *    BUTTON_UP -    Decrease the y position of the goal or robot start.
 *    BUTTON_DOWN -  Increase the y position of the goal or robot start.
 *  
 *  Primary functions:
 *    ExecuteOdometry() -       Loops through navigatingMap() to find where the robot is. Sets the robot x- and y-
 *                                coordinates to that location.
 *    Localizing() -            Localizes robot in the world.
 *    PositionVerification() -  Verifies if the robot is in a specific position in the world.
 *    LandmarkVerification() -  Verifies whether the robot is in a specific cell by checking the cells around it.
 *    MoveRobot() -             Turns the robot to the desired direction and then moves it forward approximately 18 inches.
 *    LandmarkExtraction() -    Determines the landmarks around the robot via the sensors.
 *    MakeMetricPath() -        Path planning algorithm to get to the goal location.
 *    PrintMap() -              Prints the map to the LCD; map layout depends on state.
 *  
 *  Secondary functions:
 *    SetState() -                                      State machine for setting the state between layers.
 *    updateLCD() -                                     Updates the LCD based on the state.
 *    SetGoal() -                                       Loitering state once the robot is at the goal; prints map and
 *                                                        "at goal" on LCD.
 *    FrontIRConvertToInches(float IR_Value) -          Converting function for the front IR sensor.
 *    BackIRConvertToInches(float IR_Value) -           Converting function for the back IR sensor.
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
#define LeftsonarPin TKD2
#define RightsonarPin TKD0
#define FrontIRPin TK2
#define BackIRPin TK6

// Sensor values
float sonar = 0; int frontValue = 0; int Backvalue = 0; int leftValue = 0; int rightValue = 0;

// Converted distances
float frontConv = 0; float backConv = 0; float leftConv = 0; float rightConv = 0;

// For converting functions
float IR_Value = 0; float sonarValue = 0; float sonarDistance = 0;

// Motor speeds
double mediumSpeed = 150; double fastSpeed = 175;

// Turn delays for GoToAngle()
double moveLeft = 125; double moveRight = -128;

// Robot's current position
char robotX = 0; char robotY = 0;

// Landmark extraction
char sensorNorth; char sensorSouth; char sensorWest; char sensorEast;

// Directions:
//   North = 0
//   East  = 1
//   South = 2
//   West  = 3
char currentFacing = 0; char nextDirection = 0;

// Miscellaneous 
int moveDelay                 = 1750;
boolean atGoal                = 0;
boolean landmarksVerified     = 0;
char currentLocalizationWave  = 2;
char previousLocalizationWave = 1;

// States for state machine
enum STATE {
  LocalizingRobot,
  PlanningPath,
  NavigatingPath,
  SettingGoal
};

char STATE_current = LocalizingRobot;

// Maps
//  0  = free space
//  1  = robot
// 98  = goal
// 99  = obstacle
const char rows = 6;
const char cols = 6;

// Map for navigating the robot through the world
char navigationMap[cols][rows] =
{ {99, 99, 99, 99, 99, 99},
  {99,  1, 99, 99,  1, 99},
  {99,  1, 99, 99,  1, 99},
  {99,  1,  1,  1,  1, 99},
  {99,  1, 99, 99,  1, 99},
  {99, 99, 99, 99, 99, 99}
};
// Map for path planning
char pathPlanningMap[cols][rows] =
{ {99, 99, 99, 99, 99, 99},
  {99, 98, 99, 99,  1, 99},
  {99,  1, 99, 99,  1, 99},
  {99,  1,  1,  1,  1, 99},
  {99,  1, 99, 99,  1, 99},
  {99, 99, 99, 99, 99, 99}
};

// Map for localizing the robot
char localizationMap[cols][rows] =
{ {99, 99, 99, 99, 99, 99},
  {99,  1, 99, 99,  1, 99},
  {99,  1, 99, 99,  1, 99},
  {99,  1,  1,  1,  1, 99},
  {99,  1, 99, 99,  1, 99},
  {99, 99, 99, 99, 99, 99}
};

void setup() {
  // Initialize the Robot, display, and speaker
  Robot.begin();
  Robot.beginTFT();
  Serial.begin(9600);
  // Print some text to the screen
  Robot.stroke(0, 0, 0);
  Robot.text("State", 5, 5);
}

void loop() {
  // Loop through the SetState() function
  SetState();
}

void ExecuteOdometry() {
  // Find robot coordinates
  for (char x = 0; x < rows; x++) {
    for (char y = 0; y < cols; y++) {
      if (navigationMap[y][x] == 2) { //if value in matrix is a 2
        robotX = x; //set robot x to that x value
        robotY = y; //set robot y to that y value
      }
    }
  }
}

void Localizing () {
  char landmarkCount = 0;                               // Landmark counter
  char X;                                               // X-coordinate of robot
  char Y;                                               // Y-coordinate of robot
  for (char x = 1; x < rows - 1; x++) {
    for (char y = 1; y < cols - 1; y++) {
      LandmarkVerification(y, x);                       // Checks if robot is in this position
      if (landmarksVerified == 1) {
        landmarksVerified = 0;
        X                 = x;
        Y                 = y;
        landmarkCount++;                                /  Increments variable
        delay(100);                                     // Wait 0.1 sec
        PrintMap();                                     // Print map
        delay(500);                                     // Wait 0.5 sec
      }
    }
  }
  if (landmarkCount == 1) {                             // If there is only one possible position remaining
    STATE_current         = PlanningPath;               // Changes state
    pathPlanningMap[Y][X] = 2;                          // Changes position on specified map to a 2
    navigationMap[Y][X]   = 2;                          // Changes position on specified map to a 2
  }
  else {
    for (char x = 0; x < rows; x++) {
      for (char y = 0; y < cols; y++) {
        if (localizationMap[y][x] == currentLocalizationWave) {
          robotX = x;                                   // Sets the robot's x-coordinate
          robotY = y;                                   // Sets the robot's y-coordinate
        }
      }
    }
    // Randomly choose a new direction for the robot to move
    char directionValues[] = {0, 1, 2, 3};              // Array of direction values
    // Checks if cells around the robot on the localization map are not free spaces (1's), 
    // If they are not free spaces, then changes corresponding direction's array values to -1's
    if (localizationMap[robotY - 1][robotX] != 1) {
      directionValues[0] = -1;
    }
    if (localizationMap[robotY][robotX + 1] != 1) {
      directionValues[1] = -1;
    }
    if (localizationMap[robotY + 1][robotX] != 1) {
      directionValues[2] = -1;
    }
    if (localizationMap[robotY][robotX - 1] != 1) {
      directionValues[3] = -1;
    }
    int counter = 0;                                    // Counter for counting number of directions that are free spaces
    // Determines number of cells around robot that are free spaces
    for (int i = 0; i < 4; i++) {
      if (directionValues[i] >= 0) {
        counter++;
      }
    }
    // If there are not any free spaces around the robot to navigate to, then check if the numbers are different
    if (counter == 0) {
      char directionValues[] = {0, 1, 2, 3};
      if ((localizationMap[robotY - 1][robotX] == 99) ||
          (localizationMap[robotY - 1][robotX] == previousLocalizationWave)) {
        directionValues[0] = -1;
      }
      if ((localizationMap[robotY][robotX + 1] == 99) ||
          (localizationMap[robotY][robotX + 1] == previousLocalizationWave)) {
        directionValues[1] = -1;
      }
      if ((localizationMap[robotY + 1][robotX] == 99) ||
          (localizationMap[robotY + 1][robotX] == previousLocalizationWave)) {
        directionValues[2] = -1;
      }
      if ((localizationMap[robotY][robotX - 1] == 99) ||
          (localizationMap[robotY][robotX - 1] == previousLocalizationWave)) {
        directionValues[3] = -1;
      }
      // Checks cells around robot again to see if any of them are free spaces now
      for (int i = 0; i < 4; i++) {
        if (directionValues[i] >= 0) {
          counter++;
        }
      }
    }

    int j = 0;                                          // Counter
    int openDirections[counter];                        // Array for keeping direction values that are free spaces
    // Adds the direction values that are free spaces to the openDirections[] array
    for (int i = 0; i < 4; i++) {
      if (directionValues[i] >= 0) {
        openDirections[j] = i;
        j++;
      }
    }
    nextDirection = openDirections[random(0, j - 1)];   // Randomly chooses a direction from the array for the robot to move to
    MoveRobot();                                        // Moves robot
    currentLocalizationWave++;                          // Increments variable
    previousLocalizationWave++;                         // Increments variable
  }
}

void PositionVerification() {
  LandmarkVerification(robotY, robotX);                 // Checks if robot is in current cell
  if (landmarksVerified == 0) {                         // If sensor landmarks around current cell did not match map landmarks
    navigationMap[robotY][robotX] = 1;                  // Sets position to a free space
    // Loops through cells around robot and checks if the landmarks on the sensors match the landmarks on the map
    for (char x = -1; x < 2; x++) {
      for (char y = -1; y < 2; y++) {
        LandmarkVerification(robotY + y, robotX + x);   // Checks if robot is in specified cell
        // If sensor landmarks and map landmarks match, then break
        if (landmarksVerified == 1) {
          break;
        }
     }
    }
    if (landmarksVerified == 1) {
      landmarksVerified = 0;                            // Resets flag
    }
  }
  if (landmarksVerified == 1) {                         // If flag has been set
    landmarksVerified = 0;
    char directionValues[] = {0, 1, 2, 3};              // Array of direction values
    // Checks if cells around the robot on the navigation map are not obstacles (99's), 
    // If they are not obstacles, then sets corresponding directions array index to the corresponding pathPlanningMap[] value
    if (navigationMap[robotY - 1][robotX] != 99) {
      directionValues[0] = pathPlanningMap[robotY - 1][robotX];
    }
    if (navigationMap[robotY][robotX + 1] != 99) {
      directionValues[1] = pathPlanningMap[robotY][robotX + 1];
    }
    if (navigationMap[robotY + 1][robotX] != 99) {
      directionValues[2] = pathPlanningMap[robotY + 1][robotX];
    }
    if (navigationMap[robotY][robotX - 1] != 99) {
      directionValues[3] = pathPlanningMap[robotY][robotX - 1];
    }

    char maxSurroundingCellValue = 0;                   // Holds value of cell around robot with maximum value
    char maxIndex = 0;                                  // Index of max cell
    for (char i = 0; i < 4; i++) {
      // Checks if the current max value less than the max value in the specified direction array index
      if (maxSurroundingCellValue < directionValues[i]) {
        maxSurroundingCellValue = directionValues[i];   // Updates max values
        maxIndex = i;                                   // Updates max index
      }
      // Checks if specified direction is the goal (98)
      if (directionValues[i] == 98) {
        atGoal = 1;                                     // Sets atGoal flag
        STATE_current = SettingGoal;                    // Changes the state
      }
    }
    nextDirection = maxIndex;                           // Sets the next direction for the robot to move in to the max index value

    // Checks the value of nextDirection and updates the cells on the navigationMap[] accordingly
    if (nextDirection == 0) {
      navigationMap[robotY][robotX]      = 1;
      navigationMap[robotY - 1][robotX]  = 2;
    }
    else if (nextDirection == 1) {
      navigationMap[robotY][robotX]      = 1;
      navigationMap[robotY][robotX + 1]  = 2;
    }
    else if (nextDirection == 2) {
      navigationMap[robotY][robotX]      = 1;
      navigationMap[robotY + 1][robotX]  = 2;
    }
    else {
      navigationMap[robotY][robotX]      = 1;
      navigationMap[robotY][robotX - 1]  = 2;
    }
    MoveRobot();                                        // Move the robot
  }
}

void LandmarkVerification(char yCoordinate, char xCoordinate) {
  if ((landmarksVerified                               == 0)           &&     // Checks that position hasn't yet been verified
      (navigationMap[yCoordinate][xCoordinate]         != 99)          &&     // Checks that cell is not a landmark
      (navigationMap[yCoordinate - 1][xCoordinate]     == sensorNorth) &&     // Checks North
      (navigationMap[yCoordinate][xCoordinate + 1]     == sensorEast)  &&     // Checks East
      (navigationMap[yCoordinate + 1][xCoordinate]     == sensorSouth) &&     // Checks South
      (navigationMap[yCoordinate][xCoordinate - 1]     == sensorWest)) {      // Checks West
    landmarksVerified = 1;                                                    // Flag that is set if landmarks match
    // If current state is localization state
    if (STATE_current == LocalizingRobot) {
      localizationMap[yCoordinate][xCoordinate] = currentLocalizationWave;    // Sets cell in localizationMap[] to current wave
    }
    else {
      navigationMap[yCoordinate][xCoordinate]      = 2;                       // Sets cell in naviationMap[] to 2
    }
  }
}

void MoveRobot() {
  // Checks if direction robot is facing is equal to next direction robot needs to move in, keeps looping if it isn't
  while (currentFacing != nextDirection) {
    if (currentFacing > nextDirection) {
      GoToAngle(moveLeft);                        // Turns robot to the left
      currentFacing--;                            // Decrement value
    }
    else if (currentFacing < nextDirection) {
      GoToAngle(moveRight);                       // Turns robot to the right
      currentFacing++;                            // Increment value
    }
  }
  Robot.motorsWrite(mediumSpeed, mediumSpeed);    // Move robot forward
  Robot.text("Forward", 5, 115);                  // Display text on LCD
  delay(moveDelay);                               // Delay for a time that the robot moves about 18 inches
  Robot.motorsWrite(0, 0);                        // Stop robot
}

void LandmarkExtraction() {
  if (frontConv < 9) {                            // Checks front sensor, if < 9 then obstacle, else free space
    sensorNorth = 99;
  }
  else {
    sensorNorth = 1;
  }
  if (rightConv < 9) {                            // Checks right sensor, if < 9 then obstacle, else free space
    sensorEast = 99;
  }
  else {
    sensorEast = 1;
  }
  if (backConv < 9) {                             // Checks back sensor, if < 9 then obstacle, else free space
    sensorSouth = 99;
  }
  else {
    sensorSouth = 1;
  }
  if (leftConv < 9) {                             // Checks left sensor, if < 9 then obstacle, else free space
    sensorWest = 99;
  }
  else {
    sensorWest = 1;
  }

  // Fix direction associated with landmarks based on direction robot is currently facing
  char directionPlaceholder;                      // Direction placeholder for switching direction values
  if (currentFacing == 1) {
    directionPlaceholder = sensorNorth;
    sensorNorth          = sensorWest;
    sensorWest           = sensorSouth;
    sensorSouth          = sensorEast;
    sensorEast           = directionPlaceholder;
  }
  if (currentFacing == 2) {
    directionPlaceholder = sensorNorth;
    sensorNorth          = sensorSouth;
    sensorSouth          = directionPlaceholder;
    directionPlaceholder = sensorWest;
    sensorWest           = sensorEast;
    sensorEast           = directionPlaceholder;
  }
  if (currentFacing == 3) {
    directionPlaceholder = sensorNorth;
    sensorNorth          = sensorEast;
    sensorEast           = sensorSouth;
    sensorSouth          = sensorWest;
    sensorWest           = directionPlaceholder;
  }
}

void MakeMetricPath () {
  char goalX, goalY;                                                  // Coordinates for goal position
  bool foundWave  = true;                                             // Flag for while loop
  char currentWave = 98;                                              // Looking for goal first

  // Loops through pathPlanningMap[] and plans path from goal back to robot with decreasing values
  while (foundWave == true) {
    foundWave = false;                                                // Sets flag
    for (int y = 0; y < cols; y++) {
      for (int x = 0; x < rows; x++) {
        if (pathPlanningMap[y][x] == currentWave) {
          foundWave = true;                                           // Sets flag
          goalX     = x;                                              // Sets x-coordinate of goal
          goalY     = y;                                              // Sets y-coordinate of goal

          if (goalX > 0) {                                            // Checks the array bounds to the West
            if (pathPlanningMap[goalY][goalX - 1] == 1)  {            // Checks West direction
              pathPlanningMap[goalY][goalX - 1] = currentWave - 1;
            }
          }

          if (goalX < (rows - 1)) {                                   // Checks the array bounds to the East
            if (pathPlanningMap[goalY][goalX + 1] == 1) {             // Checks East direction
              pathPlanningMap[goalY][goalX + 1] = currentWave - 1;
            }
          }

          if (goalY > 0) {                                            // Checks the array bounds to the South
            if (pathPlanningMap[goalY + 1][goalX] == 1) {             // Checks South direction
              pathPlanningMap[goalY + 1][goalX] = currentWave - 1;
            }
          }

          if (goalY < (cols - 1)) {                                   // Checks the array bounds to the North
            if (pathPlanningMap[goalY - 1][goalX] == 1) {             // Checks North direction
              pathPlanningMap[goalY - 1][goalX] = currentWave - 1;
            }
          }
        }
      }
    }
    currentWave--;                                                    // Decrements value
    delay(100);                                                       // Wait 0.1 sec
    PrintMap();                                                       // Print map
    delay(500);                                                       // Wait 0.1 sec
  }
  STATE_current = NavigatingPath;                                     // Changes state
}

void PrintMap() {
  char xx = 5;                                                        // Starting x coordinate on LCD for text
  char yy = 15;                                                       // Starting y coordinate on LCD for text
  for (char x = 0; x < rows; x++) {                                   // Scroll down the map Array
    for (char y = 0; y < cols; y++) {                                 // Scroll across the map Array
      // Checks current state of robot and updates map accordingly
      if (STATE_current == LocalizingRobot) {
        Robot.debugPrint(localizationMap[x][y], xx + 5, yy + 15);
      }
      else if (STATE_current == PlanningPath) {
        Robot.debugPrint(pathPlanningMap[x][y], xx + 5, yy + 15);
      }
      else if (STATE_current == NavigatingPath) {
        Robot.debugPrint(navigationMap[x][y], xx + 5, yy + 15);
      }
      xx = xx + 15;                                                   // Increase the next x value 15 pixels
    }
    yy = yy + 10;                                                     // Increase the next y value by 10
    xx = 5;                                                           // Set next x value to 5 pixels from the wall
  }
}

void SetState(void) {
  switch (STATE_current) {
    case LocalizingRobot:
      UpdateLCD();                                                    // Update the LCD
      ReadSensor();                                                   // Read sensors
      LandmarkExtraction();                                           // Determine landmarks around robot
      Localizing();                                                   // Localize robot
      break;
    case PlanningPath:
      UpdateLCD();                                                    // Update the LCD
      MakeMetricPath();                                               // Plan path
      break;
    case NavigatingPath:
      UpdateLCD();                                                    // Update the LCD
      ExecuteOdometry();                                              // Update robot's odometry
      ReadSensor();                                                   // Read sensors
      LandmarkExtraction();                                           // Determine landmarks around robot
      PositionVerification();                                         // Verifies position of robot
      break;
    case SettingGoal:
      UpdateLCD();                                                    // Update the LCD
      SetGoal();                                                      // Set goal
      break;
  }
}

void UpdateLCD() {
  // Clear the LCD screen
  Robot.fill(255, 255, 255);
  Robot.stroke(255, 255, 255);
  Robot.rect(5, 15, 100, 135);
  // Printing the current state to the LCD screen
  switch (STATE_current) {
    case LocalizingRobot:
      Robot.stroke(0, 0, 0);                                          // Print text
      Robot.text("Localizing", 5, 15);
      break;
    case PlanningPath:
      Robot.stroke(0, 0, 0);                                          // Print text
      Robot.text("Planning Path", 5, 15);
      break;
    case NavigatingPath:
      Robot.stroke(0, 0, 0);                                          // Print text
      Robot.text("Navigating Path", 5, 15);
      break;
    case SettingGoal:
      Robot.stroke(0, 0, 0);                                          // Print text
      Robot.text("Setting Goal", 5, 15);
      break;
  }
  delay(500);                                                         // Wait 0.5 sec
  PrintMap();                                                         // Print map
  delay(1000);                                                        // Wait 1 sec
}

void SetGoal() {
  // While the robot is in the setting goal state
  while (STATE_current == SettingGoal) {
    // Do nothing except print to LCD
    PrintMap();
    Robot.motorsWrite(0, 0);
    Robot.text("At goal", 5, 100);
  }
}

void GoToAngle(float theta) {
  int turn_delay = 0;                               // Delay for robot movement
  if (theta > 0) {                                  // Positive theta rotates robot CCW
    Robot.motorsWrite(-fastSpeed, fastSpeed);
    turn_delay = (theta + 18.833) / 0.3683;         // Delay based on excel sheet calculation of
  }                                                 //   how far the robot turned at a specified delay
  else if (theta < 0) {                             // Negative theta rotates robot CW
    Robot.motorsWrite(fastSpeed, -fastSpeed);
    turn_delay = (abs(theta) + 18.833) / 0.3683;
  }
  delay(turn_delay);                                // Delay the movement to reach the desired angle
  Robot.motorsWrite(0, 0);                          // Stops robot
  delay(250);                                       // Delay 0.25 sec
}

void ReadSensor(void) {
  // IR sensor readings not converted
  frontValue = Robot.analogRead(FrontIRPin);
  Backvalue = Robot.analogRead(BackIRPin);

  // IR sensor readings converted
  frontConv = FrontIRConvertToInches(frontValue);
  backConv = BackIRConvertToInches(Backvalue);

  int leftConvSum = 0;
  char leftConvNum = 0;
  int rightConvSum = 0;
  char rightConvNum = 0;
  for (int i = 0; i < 5; i++) {
    // Left sonar reading
    pinMode(LeftsonarPin, OUTPUT);                    // Set the PING pin as an output
    Robot.digitalWrite(LeftsonarPin, LOW);            // Set the PING pin low first
    delayMicroseconds(2);                             // Wait 2 us
    Robot.digitalWrite(LeftsonarPin, HIGH);           // Wait 5 us
    Robot.digitalWrite(LeftsonarPin, LOW);            // Set pin low first again
    pinMode(LeftsonarPin, INPUT);                     // Set pin as input with duration as reception time
    leftValue = pulseIn(LeftsonarPin, HIGH);          // Measures how long the pin is high
    leftConv  = LeftsonarConvertToInches(leftValue);
    if (leftConv >= 0) {                              // Cancels noise in left sensor
      leftConvSum += leftConv;
      leftConvNum++;
    }

    // Right sonar reading
    pinMode(RightsonarPin, OUTPUT);                   // Set the PING pin as an output
    Robot.digitalWrite(RightsonarPin, LOW);           // Set the PING pin low first
    delayMicroseconds(2);                             // Wait 2 us
    Robot.digitalWrite(RightsonarPin, HIGH);          // Trigger sonar by a 2 us HIGH PULSE
    delayMicroseconds(5);                             // Wait 5 us
    Robot.digitalWrite(RightsonarPin, LOW);           // Set pin low first again
    pinMode(RightsonarPin, INPUT);                    // Set pin as input with duration as reception time
    rightValue = pulseIn(RightsonarPin, HIGH);        // Measures how long the pin is high
    rightConv  = RightsonarConvertToInches(rightValue);
    if (rightConv >= 0) {                             // Cancels noise in right sensor
      rightConvSum += rightConv;
      rightConvNum++;
    }
  }
  leftConv = leftConvSum / leftConvNum;               // Average value of all leftConv values
  rightConv = rightConvSum / rightConvNum;            // Average value of all rightConv values
}

float FrontIRConvertToInches(float IR_Value) {
  float IRdistance = 0;
  IRdistance = 4122.3 * (pow(IR_Value, -1.12));
  return IRdistance;
}
float BackIRConvertToInches(float IR_Value) {
  float IRdistance = 0;
  IRdistance = 3597.5 * (pow(IR_Value, -1.11));
  return IRdistance;
}
float LeftsonarConvertToInches(float sonarValue) {
  float sonarDistance = 0;
  sonarDistance = (0.0069 * sonarValue) - 0.6614;
  return sonarDistance;
}
float RightsonarConvertToInches(float sonarValue) {
  sonarDistance = (0.0068 * sonarValue) - 0.7455;
  return sonarDistance;
}

