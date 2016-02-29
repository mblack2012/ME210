#include <Arduino.h>
/*************************************************************
  File:      ME210_Lab3_Part4.ino
  Contents:  This program implements Part4 of ME210 Winter '16
             Lab3.
  Notes:     Target: Arduino Uno
             Arduino IDE version: 1.6.7

  History:
  when       who  what/why
  ----       ---  ---------------------------------------------
  2016-2-7  IK   Initial version, has not been tested on HW
  2016-2-11 IK   Final version.
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

int drivespeed;

/*---------------Module Function Prototypes---*/
unsigned char TestForKey(void);
void RespToKey(void);


// FSM STATE DEFINITIONS
typedef enum{
  SPIN_CW, //spinning clockwise
  SPIN_CCW //spinning counterclockwise
} states;

/*----------------Global variables------------*/
states current_state;
states next_state;

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  //Initialize the PWM pin 
  pinMode(PWM_PIN_A, OUTPUT);
  pinMode(DIR_PIN_A, OUTPUT); 
  pinMode(PWM_PIN_B, OUTPUT);
  pinMode(DIR_PIN_B, OUTPUT); 
  pinMode(PWM_PIN_C, OUTPUT);
  pinMode(DIR_PIN_C, OUTPUT); 
  pinMode(PWM_PIN_D, OUTPUT);
  pinMode(DIR_PIN_D, OUTPUT);

  //Begin from the clockwise spinning state state
  current_state = SPIN_CW;
  next_state = SPIN_CW;
//
//  int N = 5000;
//  unsigned long starttime = millis();
//  for (int i=0; i<N; i++)
//  driveAngle(-20,0.5);
//  unsigned long endtime = millis();
//  Serial.println((endtime-starttime)/(float)N);
  driveAngle(45, 1);
}

void loop() 
{ 
  return;
}

// val is between -1 (full CCW) and 1 (full CW)
// motor is the motor number (A=1, B=2, C=3, D=4)
void driveMotor(double val, int motor) {
  switch (motor) {
    case 1: {
      if (abs(val) < 0.01) {
        digitalWrite(PWM_PIN_A, LOW); 
        break;
      }
      int speedMapping = MIN_SPEED_A + abs(val)*(MAX_SPEED-MIN_SPEED_A);
      analogWrite(PWM_PIN_A, speedMapping);
      if (val < 0) digitalWrite(DIR_PIN_A, LOW);
      else digitalWrite(DIR_PIN_A, HIGH);
//      Serial.print("A: ");
//      Serial.println(val);
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
//      Serial.print("B: ");
//      Serial.println(val);
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
//      Serial.print("C: ");
//      Serial.println(val);
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
//      Serial.print("D: ");
//      Serial.println(val);
      break;
    }
  }
}

// 0º is straight forward, 90º is right, etc. v is between 0 and 1
void driveAngle(int angleDeg, double v) {
  double angleRad = angleDeg*PI/180;
  driveMotor(sin(angleRad-3*PI/4)*v,1);
  driveMotor(sin(angleRad+PI/4)*v,2);
  driveMotor(sin(angleRad+3*PI/4)*v,3);
  driveMotor(sin(angleRad-PI/4)*v,4);
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

