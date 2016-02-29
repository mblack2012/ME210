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


//motor A
#define PWM_PIN_A 9  // pwm pin for the motor
#define DIR_PIN_A 8 //direction input to the H bridge

//motor B
#define PWM_PIN_B 11  // pwm pin for the motor
#define DIR_PIN_B 10 //direction input to the H bridge

//motor C
#define PWM_PIN_C 6  // pwm pin for the motor
#define DIR_PIN_C 7 //direction input to the H bridge

//motor D
#define PWM_PIN_D 3  // pwm pin for the motor
#define DIR_PIN_D 2 //direction input to the H bridge

#define MIN_SPEED_A 80.0
#define MIN_SPEED_B 80.0
#define MIN_SPEED_C 105.0
#define MIN_SPEED_D 80.0
#define MAX_SPEED 255


/*---------------Module Function Prototypes---*/
unsigned char TestForKey(void);
void RespToKey(void);
void spinRobot(bool direction, double speed);

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

  //Initialize the motor pins 
  pinMode(PWM_PIN_A, OUTPUT);
  pinMode(DIR_PIN_A, OUTPUT); 
  pinMode(PWM_PIN_B, OUTPUT);
  pinMode(DIR_PIN_B, OUTPUT); 
  pinMode(PWM_PIN_C, OUTPUT);
  pinMode(DIR_PIN_C, OUTPUT); 
  pinMode(PWM_PIN_D, OUTPUT);
  pinMode(DIR_PIN_D, OUTPUT);
  
  //Begin from the clockwise spinning state state
  current_state = HALT;
  next_state = HALT;
}

void loop() 
{ 
  switch(current_state) {
    case HALT:
    {

      break;
    } 
    case ORIENTING: {
      // need a way to not call this every time we loop, otherwise we will restart PWM each time
      spinRobot(true,3);
      break;
    }
    case DRIVING: {
      
      break;
    }
  }
  current_state = next_state;
  return;
} 

//Function definitions
/*----------------Module Functions--------------------------*/

// 0º is straight forward, 90º is right, etc. v is between 0 and 1
void driveAngle(int angleDeg, double v) {
  double angleRad = angleDeg*PI/180;
  driveMotor(1, sin(angleRad-3*PI/4)*v);
  driveMotor(2, sin(angleRad+PI/4)*v);
  driveMotor(3, sin(angleRad+3*PI/4)*v);
  driveMotor(4, sin(angleRad-PI/4)*v);
}

// motor is the motor number (A=1, B=2, C=3, D=4)
// val is between -1 (full CCW) and 1 (full CW)
void driveMotor(int motor, double val) {
  switch (motor) {
    case 1: {
      // if below certain threshold, we just set the motor's enable pin to LOW
      // rather than use a narrow pulse.
      if (abs(val) < 0.01) {
        digitalWrite(PWM_PIN_A, LOW); 
        break;
      }
      // gives a speed from 0 to 255, but guaranteed to be above the minimum speed
      // threshold to drive this particular motor.
      int speedMapping = MIN_SPEED_A + abs(val)*(MAX_SPEED-MIN_SPEED_A);
      analogWrite(PWM_PIN_A, speedMapping);
      if (val < 0) digitalWrite(DIR_PIN_A, LOW);
      else digitalWrite(DIR_PIN_A, HIGH);
      break;
    };
    case 2: {
      if (abs(val) < 0.01) {
        digitalWrite(PWM_PIN_B, LOW); 
        break;
      }
      int speedMapping = MIN_SPEED_B + abs(val)*(MAX_SPEED-MIN_SPEED_B);
      analogWrite(PWM_PIN_B, speedMapping);
      if (val < 0) digitalWrite(DIR_PIN_B, HIGH);
      else digitalWrite(DIR_PIN_B, LOW);
      break;
    };
    case 3: {
      if (abs(val) < 0.01) {
        digitalWrite(PWM_PIN_C, LOW); 
        break;
      }
      int speedMapping = MIN_SPEED_C + abs(val)*(MAX_SPEED-MIN_SPEED_C);
      analogWrite(PWM_PIN_C, speedMapping);
      if (val < 0) digitalWrite(DIR_PIN_C, LOW);
      else digitalWrite(DIR_PIN_C, HIGH);
      break;
    };
    case 4: {
      if (abs(val) < 0.01) {
        digitalWrite(PWM_PIN_D, LOW); 
        break;
      }
      int speedMapping = MIN_SPEED_D + abs(val)*(MAX_SPEED-MIN_SPEED_D);
      analogWrite(PWM_PIN_D, speedMapping);
      if (val < 0) digitalWrite(DIR_PIN_D, LOW);
      else digitalWrite(DIR_PIN_D, HIGH);
      break;
    }
  }
}


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

// direction should be true for CW, false for CCW
// speed between 0 and 1
void spinRobot(bool direction, double speed){
  if (direction) {
    driveMotor(1, speed);
    driveMotor(2, speed);
    driveMotor(3, speed);
    driveMotor(4, speed);
  } else {
    driveMotor(1, -speed);
    driveMotor(2, -speed);
    driveMotor(3, -speed);
    driveMotor(4, -speed);
  }
  
}
