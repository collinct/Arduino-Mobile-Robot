/* Cog-Lab02
 *  ME425-Mobile Robotics
 *  Lab 2 - Locomotion and Odometry
 *  
 *  Description:
 *  This code is used to control the robot to perform various functions via a button menu shown on the LCD screen
 *  Inputs: 5 buttons on top of robot
 *  Outputs:
 *    -go to angle: turn the robot a specific angle via the potentiometer
 *    -go to goal: rotate and move to the deisre coordinates
 *    -make circle: makes a 2-3 ft diamter circle
 *    -make square: makes a 2-3 ft side length square
 *    -make figure 8: makes a figure eight with 2-3 ft diamter circles
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

// Constants
const int LEFT = 1;
const int RIGHT = 0;
const int UP = 1;
const int DOWN = 0;
const int moveit = 2;
const int Xdir = 1;
const int Ydir = 0;
const boolean CW = true;
const boolean CCW = false;
const int selectedShape = 1;
const int selectedDirection = 0;
const float pi = 3.14159;
const int dist_calc_constant = 550; // 1000 / 1.25 ft/s (unit converstion and velocity)
const float c_b=1.003661;
const float c_l=0.997152;
const float c_r=1.0028;

// Global variables
int Xg = 0; //x goal
int Yg = 0; //y goal
int turn_delay = 0;
int directionFlag = Xdir;
int shapeFlag = selectedShape;
float theta = 0; //theta for Go-To-angle
float thetaGTG = 0;
float GTG_time = 0;
float dist = 0;
// MakeCircle Setup
int Vl = 0;
int Vr = 0;
int Circle_Time = 6500;
int dir = 0;
int slow_motor_spd = 125;
int fast_motor_spd = 200;


// Enum for storing the mode
enum mode {
  Angle,
  Goal,
  Shape
};
int Mode_current = Angle;
const int topMode = 2;
const int bottomMode = 0;

// Enum for storing the shape
enum shape {
  Square,
  Circle,
  Fig8
};
int Shape_current = Square;
const int topShape = 2;
const int bottomShape = 0;


// Define constants for moving robot
int motor_spd = 150;  //roughly 15 in/sec
boolean directionSelect = CW;
boolean flag = false;

void setup() {
  // initialize the Robot, SD card, display, and speaker
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
  Robot.text("Current Mode:", 5, 5);
  Robot.text("Action:", 5, 25);
}
void loop() {
  boolean pressed;
  //run the main menu if a button is pressed
  pressed = MainMenu(Robot.keyboardRead());
  //update LCD if button pressed
  if (pressed) {
    updateLCD();
  }
  delay(30);
}

boolean MainMenu(int keyCode) {
  switch (keyCode) {
    case BUTTON_LEFT: // increment mode between ANGLE, GOAL, SHAPE
      ChangeMode(LEFT);
      return true;
    case BUTTON_RIGHT: // increment mode between ANGLE, GOAL, SHAPE
      ChangeMode(RIGHT);
      return true;
    case BUTTON_UP: // increment angle, direction, or shape type
      ChangeMagnitude(UP);
      return true;
    case BUTTON_DOWN: // decrement angle, direction, or shape type
      ChangeMagnitude(DOWN);
      return true;
    case BUTTON_MIDDLE: // enter and move to next option and then again for sending command to robot
      if (Mode_current == Angle) {
        flag = true;
        GoToAngle(theta); //if in angle mode run the GoToAngle function
      } else if (Mode_current == Goal) { // first enter x coordinate, then enter y coordinate, then it will move on next press
        if (directionFlag == Xdir) {
          directionFlag = Ydir; //if in Xdir and press enter move to the Ydir
        } else if (directionFlag == Ydir) {
          directionFlag = moveit; //if in Ydir and press enter move to move it state
        }
        else if (directionFlag == moveit) {
          flag = true;
          GoToGoal(Xg, Yg); //if in moveit state and enter is pressed send command to robot
        }
      } else if (Mode_current == Shape) { //if in shape mode
        if (shapeFlag == selectedShape) { 
          shapeFlag = selectedDirection; //if in shape select and press enter go to direction (CW or CCW) selection
        } else if (shapeFlag == selectedDirection) {
          if (selectedShape == Square) {
            MakeSquare(directionSelect); //if selected shape is square run MakeSquare
          } else if (selectedShape == Circle) {
            MakeCircle(directionSelect); //if selected shape is circle run MakeCircle
          } else {
            MakeFigure8(directionSelect);//if selected shape is figure 8 run MakeFig8
          }
          shapeFlag = selectedShape;
        }
      }
      return true;
  }
  return false; // if no buttons pressed
}

void ChangeMode(int direction) {
  int valuechange;
  if (direction == RIGHT) { //if right button pressed go up one mode
    valuechange = 1;
  } else if (direction == LEFT) { //if left button pressed go down one mode
    valuechange = -1;
  }
  Mode_current += valuechange;

  if (Mode_current > topMode) { 
    Mode_current = bottomMode; //loop back to the first mode if the right button is pressed at the last mode
  } else if (Mode_current < bottomMode) {
    Mode_current = topMode; //loop to last mode if left button is pressed at first mode
  }
}

void ChangeMagnitude(int direction) {
  int valuechange;
  if (direction == UP) {
    valuechange = 1;
  } else if (direction == DOWN) {
    valuechange = -1;
  }
  switch (Mode_current) {
    case Angle: // increment angle in 5 degree amounts if you dont want to change via the potentiometer
      theta += 5 * valuechange;
      break;
    case Goal: // add one to Xg or Yg
      if (directionFlag == Xdir) {
        Xg += valuechange;
      } else if (directionFlag == Ydir) {
        Yg += valuechange;
      }
      break;
    case Shape: // loop through the 3 shapes
      if (shapeFlag == selectedShape) {
        Shape_current += valuechange;
        // loop back to first shape
        if (Shape_current > topShape) {
          Shape_current = bottomShape;
        } else if (Shape_current < bottomShape) { //loop up to top shape 
          Shape_current = topShape;
        }
      } else if (shapeFlag == selectedDirection) {
        directionSelect = !directionSelect;
      }
      break;
  }
}
void updateLCD() {
  // clear the LCD screen
  Robot.fill(255, 255, 255);
  Robot.stroke(255, 255, 255);
  Robot.rect(5, 15, 120, 10);
  Robot.rect(5, 35, 120, 100);

  switch (Mode_current) {
    case Angle:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Theta: ", 5, 66);
      Robot.text("Go To Angle", 5, 16);
      theta = map(Robot.knobRead(), 0, 1023, -180, 180); //read the potentiometer value
      theta = constrain(theta, -180, 180);
      Robot.debugPrint(theta, 40, 66); //print theta
      break;
    case Goal:
      Robot.stroke(0, 0, 0);
      Robot.stroke(0, 0, 0); //print and update Xg and Yg via debug print
      Robot.text("Go To Goal", 5, 16);
      Robot.text("Xg: ", 5, 46);
      Robot.debugPrint(Xg, 22, 46);
      Robot.text("Yg: ", 5, 56);
      Robot.debugPrint(Yg, 22, 56);
      break;
    case Shape:
      Robot.stroke(0, 0, 0);
      Robot.text("Shape:", 5, 16);
      if (Shape_current == Square) { //print square if square shape selected (same idea for next cases)
        Robot.text("Square", 40, 16);
      } else if (Shape_current == Circle) {
        Robot.text("Circle", 40, 16);
      } else if (Shape_current == Fig8) {
        Robot.text("Figure Eight", 40, 16);
      }
      Robot.text("Direction:", 5, 65);
      if (directionSelect == CW) { //print direction of motion
        Robot.text("CW", 65, 65);
      } else if (directionSelect == CCW) {
        Robot.text("CCW", 65, 65);
      }
      break;
  }
}

void GoToAngle(float theta) { //rotates the robot a desired angle via the potentiometer
  if (theta > 0 && flag == true) { //Positive theta rotates bot CCW
    Robot.motorsWrite(-motor_spd*c_l, motor_spd*c_r); 
    Robot.stroke(0, 0, 0);
    Robot.text("Turning!", 5, 35); //tell user the bot is turning
    turn_delay = (theta + 18.833) / 0.3683; //delay based on excel sheet calculation of how far the robot turned at a specified delay
    delay(turn_delay); //delay the movement to reach the desired angle
    Robot.motorsWrite(0, 0);
    Robot.stroke(255, 255, 255);
    Robot.text("Turning!", 5, 35);
    delay(500);
    flag = false;
  }
  else if (theta < 0 && flag == true) { //negative theta rotates bot CW
    Robot.motorsWrite(motor_spd*c_l, -motor_spd*c_r);
    Robot.stroke(0, 0, 0);
    Robot.text("Turning!", 5, 35);
    turn_delay = (abs(theta) + 18.833) / 0.3683;
    delay(turn_delay); //delay the movement to reach the desired angle
    Robot.motorsWrite(0, 0);
    Robot.stroke(255, 255, 255);
    Robot.text("Turning!", 5, 35);
    delay(500);
    flag = false;
  }
  else {
    Robot.motorsWrite(0, 0); //else dont move the bot
    flag = false;
  }
}

void GoToGoal(int Xg, int Yg) {
  thetaGTG = (atan2(Yg, Xg)) * 180 / pi; //calculate the desired theta via specified Xg and Yg
  Robot.stroke(0, 0, 0);
  Robot.text("Theta GTG: ", 5, 76);
  Robot.debugPrint(thetaGTG, 65, 76); //print the theta
  if (flag == true) {
    GoToAngle(thetaGTG); //call the GoToAngle function to rotate teh calculated angle
    delay(1000);
    flag = true;
    GoDistance(Xg, Yg); //call the GoDistance function
    flag = false;
  }

}

void GoDistance(int Xg, int Yg) { //moves the robot the calculated distance for GoToGoal
  dist = sqrt((abs(Xg) ^ 2) + (abs(Yg) ^ 2))*c_b; //distance needed to travel 
  Robot.stroke(0, 0, 0);
  Robot.text("Dist: ", 5, 86);
  Robot.debugPrint(dist, 40, 86); //print distance
  GTG_time = dist * dist_calc_constant; //
  Robot.text("Delay:", 5, 96);
  Robot.debugPrint(GTG_time, 40, 96); //print time 
  Robot.motorsWrite(motor_spd, motor_spd); //move the bot forward
  Robot.stroke(0, 0, 0);
  Robot.text("MOVING!", 5, 35);
  delay(GTG_time); //delay the movement for the calculated time
  Robot.motorsWrite(0, 0);
  Robot.stroke(255, 255, 255);
  Robot.text("MOVING!", 5, 35);
  flag = false;
}

void MakeSquare(boolean CW) {
  int direction = -1;
  if (CW) { //change the direction if its not CCW
    direction = 1;
  }
  flag = true;
  GoToGoal(direction*2, 0 ); //move the first length
  delay(1000);
  flag = true;
  GoToGoal(0, direction*2); //move second length
  delay(1000);
  flag = true;
  GoToGoal(0, direction*2); //move third length
  delay(1000);
  flag = true;
  GoToGoal(0, direction*2 ); //move the fourth length
  delay(1000);
}
void MakeCircle(boolean CW) {
  if (CW) { 
    Robot.motorsWrite(fast_motor_spd, slow_motor_spd); //move the bot in a CW circle
  }
  else {
    Robot.motorsWrite(slow_motor_spd, fast_motor_spd); //move the bot in a CCW circle
  }
  Robot.stroke(0, 0, 0);
  Robot.text("CIRCLING!", 5, 35);
  if (CW) {
    Circle_Time = 6500; //time delay for CW circle
  }
  else
    Circle_Time = 8000; //time delay for CCW circle
  delay(Circle_Time); //delay for the desired time
  Robot.motorsWrite(0, 0);
  Robot.stroke(255, 255, 255);
  Robot.text("CIRCLING!", 5, 35);
}
void MakeFigure8(boolean direction) { //makes a circle by calling the circel function twice. 
  MakeCircle(direction); // right 
  MakeCircle(!direction); // left
  Robot.motorsWrite(0, 0);
}





