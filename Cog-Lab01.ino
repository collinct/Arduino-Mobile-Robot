/* Cog-Lab01
 *  ME425-Mobile Robotics
 *  Lab 1 - Teleoperation via Remote Control
 *  
 *  Description:
 *  This code is used to control the robot to move via an IR remote. 
 *  It can make the robot move forward/backward, turn left/right, and spin left/right.
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
#include <IRremote.h>
#include <ArduinoRobot.h>
#include <Wire.h>
#include <SPI.h>

// Define a few commands from remote control 
#define IR_CODE_FORWARD 16621663      //UP ARROW
#define IR_CODE_BACKWARD 16625743    //DOWN ARROW
#define IR_CODE_TURN_LEFT 16584943   //LEFT ARROW
#define IR_CODE_TURN_RIGHT 16601263   //RIGHT ARROW
#define IR_CODE_SPIN_LEFT 16582903   //1
#define IR_CODE_SPIN_RIGHT 16599223   //3




int RECV_PIN = TKD1; // the pin the IR receiver is connected to
IRrecv irrecv(RECV_PIN); // an instance of the IR receiver object
decode_results results; // container for received IR codes
//define constants for moving robot
int move_time = 500;
int turn_time = 250;
int spin_time = 250;
int motor_spd = 200;
//used for the turn left/right
int slow_motor_spd = 100;
int fast_motor_spd = 200;

void setup() {
  // initialize the Robot, SD card, display, and speaker
  Robot.begin();
  Robot.beginTFT();
  Robot.beginSD();
  Serial.begin(9600);
  // print some text to the screen
  Robot.stroke(0, 0, 0);
  Robot.text("Remote Control Code:", 5, 5);
  Robot.text("Command:", 5, 26);
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {

  if (irrecv.decode(&results)) {
    Robot.text("code received", 5, 100);
    processResult(); // if there is an IR command, process it
    irrecv.resume(); // resume receiver
  }
}

void processResult() {
  unsigned long res = results.value;
  // print the value to the screen
  Robot.debugPrint(res, 5, 15);
  Serial.println(res, HEX);
  //setup if statement to check if a registered button on the IR remote was pressed
  if (res == IR_CODE_FORWARD || res == IR_CODE_BACKWARD || res == IR_CODE_TURN_LEFT || res == IR_CODE_TURN_RIGHT || res == IR_CODE_SPIN_RIGHT || res == IR_CODE_SPIN_RIGHT )
  {
    Robot.fill(255, 255, 255);
    Robot.stroke(255, 255, 255);
    Robot.rect(5, 36, 55, 10);
  }
  //switch statements for each IR button case
  switch (results.value) {
    //forward arrow press
    case (IR_CODE_FORWARD):
      Robot.stroke(0, 0, 0);
      Robot.text("Forward", 5, 36);
      Robot.motorsWrite(motor_spd, motor_spd);
      delay(move_time);
      Robot.motorsStop();
      break;
  //back arrow press
    case (IR_CODE_BACKWARD):
      Robot.stroke(0, 0, 0);
      Robot.text("Backwards", 5, 36);
      Robot.motorsWrite(-motor_spd, -motor_spd);
      delay(move_time);
      Robot.motorsStop();
      break;
  //1 button pressed
    case (IR_CODE_SPIN_LEFT):
      Robot.stroke(0, 0, 0);
      Robot.text("Spin Left", 5, 36);
      Robot.motorsWrite(-motor_spd, motor_spd);
      delay(spin_time);
      Robot.motorsStop();
      break;
   //3 button pressed
    case (IR_CODE_SPIN_RIGHT):
      Robot.stroke(0, 0, 0);
      Robot.text("Spin Right", 5, 36);
      Robot.motorsWrite(motor_spd, -motor_spd);
      delay(spin_time);
      Robot.motorsStop();
      break;
  //left arrow pressed
    case (IR_CODE_TURN_LEFT):
      Robot.stroke(0, 0, 0);
      Robot.text("Turn Left", 5, 36);
      Robot.motorsWrite(slow_motor_spd, fast_motor_spd);
      delay(turn_time);
      Robot.motorsStop();
      break;
  //right arrow pressed
    case (IR_CODE_TURN_RIGHT):
      Robot.stroke(0, 0, 0);
      Robot.text("Turn Right", 5, 36);
      Robot.motorsWrite(fast_motor_spd, slow_motor_spd);
      delay(turn_time);
      Robot.motorsStop();
      break;

  }
}

