/* Cog-Lab05
 *  ME425-Mobile Robotics
 *  Lab 5 - Light Sensing
 *  
 *  Description:
 *  This code is used to control the robot to follow a light via 4 behaviors that reflect the Braitenberg vehicle
 *  beaviors: Fear, agression, love, and explore.  Obstacle avoidance was also implemented, allowing the robot to 
 *  stop if an obstacle is detected by the front sensor.
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
#define RightPhotoPin TK1
#define FrontIRPin TK2

// Sensor values
int Frontvalue = 0;
int Leftvalue = 0;
int Rightvalue = 0;
// Photo distances
float Leftconv = 0;
float Rightconv = 0;
//for converting functions
float IRvalue = 0;
//IR distances
float IRdistance = 0;
float Frontconv = 0;

// Min and max values for photoresistors
int leftMin = 820;
int leftMax = 1023;
int rightMin = 890;
int rightMax = 920;
double speedMin = 80;
double speedMax = 150;

//avoid flag for the option of obstacle avoidance
boolean Avoid = false;
boolean pressed;

//enum for the mode selection
enum mode {
  Ready,
  Agression
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
  Robot.text("Mode:", 5, 5);

}

void loop() {
  ReadSensor();
  //run the state machine if a button is pressed
  pressed = SetState(Robot.keyboardRead());
  //update LCD if button pressed
  if (pressed) {
    updateLCD();
  }
}

void ReadSensor(void) {
  //read the values from the sensors
  Leftvalue = Robot.analogRead(LeftPhotoPin);
  Rightvalue = Robot.analogRead(RightPhotoPin);
  Frontvalue = Robot.analogRead(FrontIRPin);
  //convert distances to inches
  Frontconv = FrontIRConvertToInches(Frontvalue);
  delay(100);

}

void ReadyState() {
  while (STATE_current = Ready) {
    //do nothing except read buttons
    Robot.keyboardRead();
  }
}

void AgressionMove(void) {
  while (STATE_current = Agression) {
    //while in this state read buttons and sensors
    Robot.keyboardRead();
    ReadSensor();
    //if the button_up is pressed again turn on obstacle avoidance
    if (Robot.keyboardRead() == BUTTON_UP) {
      Avoid = true;
      Robot.text("Avoid", 5, 25);
    }
    if (Frontconv < 5) { //if the front sensor detects an obstacle
      if (Avoid == true) { //if it is in obstacle avoidance back the robot up and stop
        Robot.motorsWrite(0, 0);
        delay(100);
        Robot.motorsWrite(-100, -100);
        delay(500);
        Robot.motorsWrite(0, 0);
        Leftconv = 0;
        Rightconv = 0;
      }
      else {
        //if no avoidance, ignore the obstacle
      }
    }
    else {
      //if both are smaller than the min then dont move the bot
      if ((Leftvalue < leftMin) && (Rightvalue < rightMin)) {
        Leftconv = 0;
        Rightconv = 0;
      }
      //if the bot is close enough to the flashlight, stop the bot
      else if ((Leftvalue >= (leftMax - 5)) || (Rightvalue >= (rightMax - 5))) {
        Leftconv = 0;
        Rightconv = 0;
      }
      //if a right value is detected and not left
      else if ((Leftvalue < leftMin) && (Rightvalue >= rightMin )) {
        //set left value to the minimum
        Leftvalue = leftMin;
        Leftconv = map(Leftvalue, leftMin, leftMax, speedMin, speedMax);
        Rightconv = map(Rightvalue, rightMin, rightMax, speedMin, speedMax);
      }
      //if left value is detected and not right
      else if ((Leftvalue >= leftMin) && (Rightvalue < rightMin )) {
        //set right value to minimum
        Rightvalue = rightMin;
        Leftconv = map(Leftvalue, leftMin , leftMax, speedMin, speedMax);
        Rightconv = map(Rightvalue, rightMin , rightMax, speedMin, speedMax);
      }
      else {
        //else just map the values
        Leftconv = map(Leftvalue, leftMin , leftMax, speedMin, speedMax);
        Rightconv = map(Rightvalue, rightMin , rightMax, speedMin, speedMax);
      }
    }
    //move the bot based on the if statements' output
    Robot.motorsWrite(Rightconv, Leftconv);
    //if the middle button is pressed set the state to ready and go to state machine and break out of loop
    if (Robot.keyboardRead() == BUTTON_MIDDLE) {
      Robot.motorsWrite(0, 0);
      delay(100);
      SetState(Robot.keyboardRead());
      break;
    }
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
      STATE_current = Agression; //if up button is pressed go to agression state
      updateLCD(); //update the LCD
      AgressionMove();
      return true;
  }
  return false; // if no buttons pressed
}

void updateLCD() {
  // clear the LCD screen
  Robot.fill(255, 255, 255);
  Robot.stroke(255, 255, 255);
  Robot.rect(5, 15, 60, 20);
  //printing the current state to the LCD screen
  switch (STATE_current) {
    case Ready:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Ready", 5, 15);
      break;
    case Agression:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Agression", 5, 15);
      break;
  }
}
//converting function for the front sensor.
float FrontIRConvertToInches(float IRValue) {
  IRdistance = 4122.3 * (pow(IRValue, -1.12));
  return IRdistance;
}








