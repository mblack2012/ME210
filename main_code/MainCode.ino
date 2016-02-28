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
  current_state = SPIN_CW;
  next_state = SPIN_CW;
}

void loop() 
{ 
  switch(current_state)
  {
   /*
   ** HALT STATE
   */ 
   case SPIN_CW:
   {
      //Serial.println("SPIN CW");
      //upon a keypress, go to the counterclockwise spinning state
      if (TestForKey()){
        next_state = SPIN_CCW;
        RespToKey();
      }
      else next_state = SPIN_CW;
      
      val = 1023;            // reads the value of the potentiometer (value between 0 and 1023) 
      //Serial.println(val);
      val = map(val, 0, 1023, 0, 255);     // scale it to use it with the PWM library
      analogWrite(pwmpin1,val);
      analogWrite(pwmpin2,val);
      analogWrite(pwmpin3,val);
      analogWrite(pwmpin4,val);
      digitalWrite(direction_pin,LOW);
      digitalWrite(direction_pin2,HIGH);
      digitalWrite(direction_pin3,LOW);
      digitalWrite(direction_pin4,LOW);
      break;
   }
    case SPIN_CCW:
    {
      //Serial.println("SPIN CCW");
      //upon a keypress, go to the clockwise spinning state
      if (TestForKey()){
        next_state = SPIN_CW;
        RespToKey();
      }
      else next_state = SPIN_CCW;
      
      val = 1023;            // reads the value of the potentiometer (value between 0 and 1023) 
      //Serial.println(val);
      val = map(val, 0, 1023, 0, 255);     // scale it to use it with the PWM library
      analogWrite(pwmpin1,val);
      analogWrite(pwmpin2,val);
      analogWrite(pwmpin3,val);
      analogWrite(pwmpin4,val);
      digitalWrite(direction_pin,HIGH);
      digitalWrite(direction_pin2,HIGH);
      digitalWrite(direction_pin3,HIGH);
      digitalWrite(direction_pin4,LOW);
      break;
    }
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

