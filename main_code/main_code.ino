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

#define BAUD_RATE 115200
#define ULTRASONIC_SERIAL Serial1
#define MAX_SERIAL_LEN 20
#define NUM_SERIAL_TOKENS 7

#define USONIC_RIGHT1_IDX 0
#define USONIC_RIGHT2_IDX 1
#define USONIC_BACK1_IDX 2
#define USONIC_BACK2_IDX 3
#define USONIC_LEFT_IDX 4
#define USONIC_RIGHTANGLE_IDX 5
#define USONIC_BACKANGLE_IDX 6

#define START_RIGHTDISTANCE_MAX 50 // cm
#define START_BACKDISTANCE_MAX 50 // cm

#define ARENA_HALFWIDTH 122 // cm
#define ROBOT_LENGTH 28 // cm



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

// current coordinates, defined relative to back left corner of bot. x=0 is middle of arena, y=0 is back
int x; // cm
int y; // cm

void setup() {
  Serial.begin(9600);
  ULTRASONIC_SERIAL.begin(BAUD_RATE);

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
  current_state = ORIENTING;
  next_state = ORIENTING;
  delay(300);
  spinRobot(true,0.5);

  //TODO: START TIMER

  x = 100; // initial guess
  y = 20; // initial guess
}

void loop() 
{
  if (checkTimeup()) {
    halt();
    next_state = HALT;
  }
  
  if (current_state != ORIENTING) {
    int* values = parseString(readUltrasonicSerial());
    setBiases(values);
    setXY(values);
  }
  
  switch(current_state) {
    case HALT:
    {
      break;
    } 
    case ORIENTING: {
      if (checkOriented()) {
        driveToFirstBucket();
        next_state = DRIVING;
      }
      break;
    }
    case DRIVING: {
      if (checkTape()) {
        stopDriving();
        startDumping();
        next_state = DUMPING;
      }
      break;
    }
    case DUMPING: {
      if (checkDoneDumping()) {
        startReturning();
        next_state = RETURNING;
      }
      break;
    }
    case RETURNING: {
      if (checkDoneReturning()) {
        stopDriving();
        next_state = LOADING;
      }
      break;
    } case LOADING: {
      if (checkLoaded()) {
        driveToBucket();
        next_state = DRIVING;
      }
      break;
    }
  }
  current_state = next_state;
  return;
} 

//Function definitions
/*----------------Module Functions--------------------------*/

void halt() {
  stopDriving();
  stopDumping();
}

void stopDriving() {
  digitalWrite(PWM_PIN_A, LOW);
  digitalWrite(PWM_PIN_B, LOW);
  digitalWrite(PWM_PIN_C, LOW);
  digitalWrite(PWM_PIN_D, LOW);
}

void startDumping() {
  
}

void stopDumping() {
  
}

void driveToBucket() {
  
}

void driveToFirstBucket() {
  stopDriving();
}

void startReturning() {
  
}

bool checkTape() {
  return false;
}

bool checkLoaded() {
  return false;
}

bool checkDoneReturning() {
  return false;
}

bool checkDoneDumping() {
  return false;
}

bool checkTimeup() {
  return false;
}

bool checkOriented() {
  int* values = parseString(readUltrasonicSerial());
  
  if (values[USONIC_RIGHTANGLE_IDX] == 0 
      && values[USONIC_BACKANGLE_IDX] == 0 
      && values[USONIC_RIGHT1_IDX] < START_RIGHTDISTANCE_MAX
      && values[USONIC_RIGHT2_IDX] < START_RIGHTDISTANCE_MAX
      && values[USONIC_BACK1_IDX] < START_BACKDISTANCE_MAX
      && values[USONIC_BACK1_IDX] < START_BACKDISTANCE_MAX
      && values[USONIC_BACK2_IDX] < START_BACKDISTANCE_MAX) {
    setXY(values);
    return true;
  }
  return false;
}

void setBiases(int* values) {
  
}

void setXY(int* values) {
  int rightdistance = values[USONIC_RIGHT1_IDX];
  if (values[USONIC_RIGHT1_IDX] != -1) {
    if (values[USONIC_RIGHT2_IDX] != -1) {
      rightdistance = (values[USONIC_RIGHT1_IDX] + values[USONIC_RIGHT2_IDX])/2;
    } else {
      rightdistance = values[USONIC_RIGHT1_IDX];
    }
  } else {
    if (values[USONIC_RIGHT2_IDX] != -1) {
      rightdistance = values[USONIC_RIGHT2_IDX];
    } else {
      rightdistance = -1;
    }
  }
  int leftdistance = values[USONIC_LEFT_IDX];
  
  if (rightdistance != -1) {
    if (leftdistance != -1) {
      x = (rightdistance+ROBOT_LENGTH + leftdistance)/2;
    } else {
      x = rightdistance+ROBOT_LENGTH;
    }
  } else if (leftdistance != -1) {
      x = leftdistance;
  }
  
  if (values[USONIC_BACK1_IDX] > values[USONIC_BACK2_IDX] && values[USONIC_BACK1_IDX] != -1) {
    y = values[USONIC_BACK1_IDX];
  } else if (values[USONIC_BACK2_IDX] != -1) {
    y = values[USONIC_BACK2_IDX];
  }
}

int* parseString(char* str) {
  int values[NUM_SERIAL_TOKENS] = {-1,-1,-1,-1,-1,-1,-1};
  int count = 0;
  char* token = strtok(str," ");
  while (token != NULL && count < NUM_SERIAL_TOKENS) {
    values[count] = atoi(token);
    token = strtok(NULL, " ");
    count++;
  }
  return values;
}

char* processIncomingByte (const byte inByte) {
  static char input_line [MAX_SERIAL_LEN];
  static unsigned int input_pos = 0;

  switch (inByte) {

    case '\n':
      input_line [input_pos] = 0;
      
      // reset buffer for next time
      input_pos = 0;
      return input_line;
//      process_data (input_line);
      break;
    case '\r':   // discard carriage return
      break;
    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_SERIAL_LEN - 1))
        input_line [input_pos++] = inByte;
      break;
  }
  return "";
}

char* readUltrasonicSerial() {
  char* ultrasonicResult;
  if (!ULTRASONIC_SERIAL.available()) return "";
  
  // once all bytes have been read, result will contain the resulting string
  while (ULTRASONIC_SERIAL.available()) {
    ultrasonicResult = processIncomingByte(ULTRASONIC_SERIAL.read());
  }
  ULTRASONIC_SERIAL.flush();
  
  return ultrasonicResult;
}


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
