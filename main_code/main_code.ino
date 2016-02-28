#include <Arduino.h>
/*************************************************************
  File:      ME210_Project_MainCode.ino
  Contents:  Overall firmware for the ME210 Winter 2016 B3K1 Team
  Notes:     Target: Arduino Uno
             Arduino IDE version: 1.6.7

  History:
  when       who  what/why
  ----       ---  ---------------------------------------------
  2016-2-28  IK   Initial skeleton version
 ************************************************************/

/*---------------Module Function Prototypes---*/
unsigned char TestForKey(void);
void RespToKey(void);
void spinRobot(bool directoin, int speed);

// FSM STATE DEFINITIONS
typedef enum{
  HALT, //stops the robot
  ORIENTING, //spin and orient the robot and determine the initial x,y coordinates
  DRIVING, //drives the robot 
  RETURNING, //return the robot to the base
  DUMPING, //dump chips
  LOADING //the robot waits until the chips are loaded
} states;

/*----------------Global variables------------*/
states current_state;
states next_state;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("ME210 Lab3 Part 4, Kushan, Breed");

  //Begin from the clockwise spinning state state
  current_state = HALT;
  next_state = HALT;
}

void loop() 
{ 
  switch(current_state)
  {
   /*
   ** HALT STATE
   */
   case HALT:
   {

      break;
   } 
   case ORIENTING:
   {
      spinRobot(true,3);
      break;
   }
   /*
    case DRIVING:
    {

      break;
    }
    */
  }
  current_state = next_state;
  return;
} 

//Function defintions
/*----------------Module Functions--------------------------*/
unsigned char TestForKey(void) {
  unsigned char KeyEventOccurred;
  
  KeyEventOccurred = Serial.available();
  return char(KeyEventOccurred);
}

void RespToKey(void) {
  unsigned char theKey;
  
  theKey = Serial.read();
  
  Serial.write(theKey);
  Serial.print(", ASCII=");
  Serial.println(theKey,HEX);
}

void spinRobot(bool direction, int speed){
  //does nothing as of now
}
