/* Cog-Lab03
 *  ME425-Mobile Robotics
 *  Lab 3 - Random Wander & Obstacle Avoidance
 *  
 *  Description:
 *  This code is used to control the robot from a menu to randomly wander, avoid obstacles via a shy behavior 
 *  and an agressive behavior, as well as randomly wander with a shy behavior avoidance.
 *  
 *  Authors:
 *  Chris Collinsworth
 *  Jordan Patterson
 *  
 *  Latest Revision Date:
 *  05/18/2018
 ************************************************************************
 */

// include the necessary libraries
#include <ArduinoRobot.h>
#include <Wire.h>
#include <SPI.h>
#include <Math.h>

// Sensor Setup
#define LeftSonarPin TKD2
#define RightSonarPin TKD0
#define FrontIRPin TK2
#define BackIRPin TK6

//constants
const int LEFT = 1;
const int RIGHT = 0;
const int UP = 1;
const int DOWN = 0;
const float pi = 3.14159;    

//converted sensor values
int Leftvalue = 0;
int Rightvalue = 0;
int Frontvalue = 0;
int Backvalue = 0;

//for converting functions
float IRvalue = 0;
float Sonarvalue = 0;
//IR distances
float IRdistance = 0;
float Frontconv = 0;
float Backconv = 0;
//sonar distances
float Sonardistance = 0;
float Leftconv = 0;
float Rightconv = 0;


int motor_spd = 200;              // roughly 20 in/sec
int minDistance = 3;              //3 in min range for sensor reading
int maxDistance = 8;              //8 in max range for sensor reading
int theta = 0;                    //setup theta for the turning

//setup for the menu
boolean ShyFlag = false;
enum mode {
  Shy,
  Agressive,
  RandomNone,
  Smart
};
int Mode_current = Shy ;
const int topMode = 2;
const int bottomMode = 0;

void setup() {
  // initialize the Robot, display, and speaker
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSpeaker();
  Serial.begin(9600);
//  print some text to the screen
  Robot.stroke(0, 0, 0);
  Robot.text("Mode:", 5, 5);
}
void loop() {

  boolean pressed;
  //run the main menu if a button is pressed
  pressed = MainMenu(Robot.keyboardRead());
  //update LCD if button pressed
  if (pressed) {
    updateLCD();
  }
 
}
//functions for converting the sonar and IR values and converting them to a distance in inches
float FrontIRConvertToInches(float IRValue) {
  IRdistance = 4122.3 * (pow(IRValue, -1.12));
  return IRdistance;
}
float BackIRConvertToInches(float IRValue) {
  IRdistance = 3597.5 * (pow(IRValue, -1.11));
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

//main menu function
boolean MainMenu(int keyCode) {
  switch (keyCode) {
    case BUTTON_LEFT: // increment mode between RANDOM WANDER, SHY KID, AGRESSIVE KID, and SMART WANDER
      ChangeMode(LEFT);
      return true;
    case BUTTON_RIGHT: // increment mode between RANDOM WANDER, SHY KID, AGRESSIVE KID, and SMART WANDER
      ChangeMode(RIGHT);
      return true;
    case BUTTON_MIDDLE: // enter and move to next option and then again for sending command to robot
      if (Mode_current == Shy) { 
        ShyKid(); //run shy kid if mode selected is shy 
      } else if (Mode_current == Agressive) {
        AggressiveKid(); //run agressive kid if agressive mode is selected
      } if (Mode_current == RandomNone) {
        Random_sub(); //if in RandomNone, run random_sub funcion with no avoidance
      }
      if (Mode_current == Smart) {
        ShyFlag = true; //set the shy flag to true so that if differs from the regular shy kid 
        ShyKid(); //run the shy kid function with avoidance and random wander
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
}
void updateLCD() {
  // clear the LCD screen
  Robot.fill(255, 255, 255);
  Robot.stroke(255, 255, 255);
  Robot.rect(5, 15, 120, 10);
  Robot.rect(5, 35, 120, 100);
//printing the current mode to the LCD screen
  switch (Mode_current) { 
    case Shy:
      Robot.stroke(0, 0, 0); //print text
      Robot.text("Shy", 5, 16);
      break;
    case Agressive:
      Robot.stroke(0, 0, 0); 
      Robot.text("Agressive", 5, 16);
      break;
    case RandomNone:
      Robot.stroke(0, 0, 0);
      Robot.text("None", 5, 16);
      break;
    case Smart:
      Robot.stroke(0, 0, 0);
      Robot.text("Smart", 5, 16);
      break;
  }
}
void AggressiveKid() {
  while (1) {
    ReadSensor(); //read the sensors
    //if the front sensor reading is between 3 and 8 inches stop and beep
    if (Frontconv < maxDistance && Frontconv > minDistance) {
      Robot.motorsWrite(0, 0);
      Robot.beep(BEEP_SIMPLE);
    }
    else
      //if not in range move robot forward
      Robot.motorsWrite(motor_spd, motor_spd);
  }
}
void ShyKid() {
  while (1) {
    ReadSensor(); //read the sensors
    int x = Rightconv - Leftconv;
    int y = Frontconv - Backconv;
    //if the sensor distances are beyond 10 inches, randomly wander
    if (Rightconv > 10 && Leftconv > 10 && Frontconv > 10 && Backconv > 10) {
      //if smart wander is on run random wander
      if (ShyFlag == true) {
        RandomWander();
      }
      else {
        //if regular shy kid stop the robot 
        Robot.motorsWrite(0, 0);
      }
    }
    else if (x >= 0 && y >= 0) {                                 // x and y are both positive
      theta = (atan2(x, y)) * 180 / pi;
      Robot.motorsWrite(motor_spd, motor_spd * (theta / 90)); // turn opposite
      Robot.beep(BEEP_SIMPLE);
      delay(250);
    }
    else if (Frontconv < maxDistance && Frontconv > minDistance) {
      Robot.beep(BEEP_SIMPLE);
      Robot.motorsWrite(0, 0);
      Robot.turn(90);
    }
    else if (x < 0 && y < 0) {                              // x and y are both negative
      theta = (atan2(x, y)) * 180 / pi;
      Robot.motorsWrite(motor_spd * (theta / 180), motor_spd); //turn opposite
      Robot.beep(BEEP_SIMPLE);
      delay(250);
    }
    else if (x >= 0 && y < 0) {                             // x is positive, y is negative
      theta = (atan2(x, y)) * 180 / pi;
      Robot.motorsWrite(motor_spd, motor_spd * (theta / 180)); //turn opposite
      Robot.beep(BEEP_SIMPLE);
      delay(250);
    }
    else if (x < 0 && y >= 0) {                                                 // x is negative, y is positive
      theta = (atan2(x, y)) * 180 / pi;
      Robot.motorsWrite(motor_spd * (theta / 90), motor_spd); // turn opposite
      Robot.beep(BEEP_SIMPLE);
      delay(250);
    }
  }
}
void RandomWander() {
  int LEFT_MOTOR_SPEED = 0;       //
  int RIGHT_MOTOR_SPEED = 0;       //
  //while range is beyond 10 randomly wander
  while (Rightconv > 10 && Leftconv > 10 && Frontconv > 10 && Backconv > 10) {
    ReadSensor(); //read the sensors
   //randomly move the robot at different motor speeds
    LEFT_MOTOR_SPEED = random(-55, 255);
    RIGHT_MOTOR_SPEED = random(-55, 255);
    Robot.motorsWrite(LEFT_MOTOR_SPEED, RIGHT_MOTOR_SPEED);
    //if any of the sensor seadings are within th 3-8 range, go back to shy kid function to avoid the obstacle
    if ((Frontconv < maxDistance && Frontconv > minDistance) ||
        (Backconv < maxDistance && Backconv > minDistance) ||
        (Leftconv < maxDistance && Leftconv > minDistance) ||
        (Rightconv < maxDistance && Rightconv > minDistance)) {
      ShyKid();
    }
    delay(250);
  }
}
void Random_sub(void) {
  int LEFT_MOTOR_SPEED = 0;       //
  int RIGHT_MOTOR_SPEED = 0;       //
  while (1) {
    //randomly move the robot at different motor speeds with no avoidance
    LEFT_MOTOR_SPEED = random(-255, 255);
    RIGHT_MOTOR_SPEED = random(-255, 255);
    Robot.motorsWrite(LEFT_MOTOR_SPEED, RIGHT_MOTOR_SPEED);
    delay(500);
  }
}
void ReadSensor(void) {
  //IR sensor readings not converted
  Frontvalue = Robot.analogRead(FrontIRPin);
  Backvalue = Robot.analogRead(BackIRPin);
  //convert IR readings to inches via the called function
  Frontconv = FrontIRConvertToInches(Frontvalue);
  Backconv = BackIRConvertToInches(Backvalue);
  
  //left sonar reading
  pinMode(LeftSonarPin, OUTPUT);//set the PING pin as an output
  Robot.digitalWrite(LeftSonarPin, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  Robot.digitalWrite(LeftSonarPin, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  Robot.digitalWrite(LeftSonarPin, LOW);//set pin low first again
  pinMode(LeftSonarPin, INPUT);//set pin as input with duration as reception time
  Leftvalue = pulseIn(LeftSonarPin, HIGH);//measures how long the pin is high
  Leftconv = LeftSonarConvertToInches(Leftvalue); //convert left sonar reading to inches via the called function

  //right sonar reading
  pinMode(RightSonarPin, OUTPUT);//set the PING pin as an output
  Robot.digitalWrite(RightSonarPin, LOW);//set the PING pin low first
  delayMicroseconds(2);//wait 2 us
  Robot.digitalWrite(RightSonarPin, HIGH);//trigger sonar by a 2 us HIGH PULSE
  delayMicroseconds(5);//wait 5 us
  Robot.digitalWrite(RightSonarPin, LOW);//set pin low first again
  pinMode(RightSonarPin, INPUT);//set pin as input with duration as reception time
  Rightvalue = pulseIn(RightSonarPin, HIGH);//measures how long the pin is high
  Rightconv = RightSonarConvertToInches(Rightvalue); //convert left sonar reading to inches via the called function
}









