#include <Arduino.h>
#include <Wire.h>
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

// 0-4 are the five front IRs from left to right. 5 is the back IR. 
#define IR_PIN0 40
#define IR_PIN1 41
#define IR_PIN2 42
#define IR_PIN3 43
#define IR_PIN4 44
#define IR_PIN5 45

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

#define MAX_SERIAL_LEN 20
#define NUM_SERIAL_TOKENS 7

#define USONIC_RIGHT1_IDX 0
#define USONIC_RIGHT2_IDX 1
#define USONIC_BACK1_IDX 2
#define USONIC_BACK2_IDX 3
#define USONIC_LEFT_IDX 4
#define USONIC_RIGHTANGLE_IDX 5
#define USONIC_BACKANGLE_IDX 6

#define START_RIGHTDISTANCE_MAX 61 // cm
#define START_BACKDISTANCE_MAX 61 // cm

#define ARENA_HALFWIDTH 122 // cm
#define ROBOT_LENGTH 28 // cm

// half width of the back IR block
#define IR_BLOCK_MARGIN 8 // cm

// assumed y thickness of the IR block
#define MIN_Y 8 // cm

// assumed Y coord of back of robot when we dump
#define MAX_Y 68 // cm

// coordinates of the centers of the buckets, from left to right
#define BUCKET1_X -81 // cm (-32in)
#define BUCKET2_X -41 // cm (-16in)
#define BUCKET3_X  -0 // cm (0in)
#define BUCKET4_X  41 // cm (16in)
#define BUCKET5_X  81 // cm (32in)



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

// current coordinates, defined relative to middle back side of bot. x=0 is middle of arena, y=0 is back
int x; // cm
int y; // cm

double motorABias;
double motorBBias;
double motorCBias;
double motorDBias;

int* usonicValues;

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  
  Serial.begin(9600);           // start serial for output
  
  usonicValues = (int*)malloc(sizeof(int)*7);
  for (int i=0; i<7; i++) {
    usonicValues[i] = -1;
  }
  
  //Initialize the motor pins 
  pinMode(PWM_PIN_A, OUTPUT);
  pinMode(DIR_PIN_A, OUTPUT); 
  pinMode(PWM_PIN_B, OUTPUT);
  pinMode(DIR_PIN_B, OUTPUT); 
  pinMode(PWM_PIN_C, OUTPUT);
  pinMode(DIR_PIN_C, OUTPUT); 
  pinMode(PWM_PIN_D, OUTPUT);
  pinMode(DIR_PIN_D, OUTPUT);
  
  //Begin from the clockwise spinning state
  current_state = ORIENTING;
  next_state = ORIENTING;
  delay(300);

  x = 90; // initial guess
  y = 20; // initial guess

  motorABias = 1.0;
  motorBBias = 1.0;
  motorCBias = 1.0;
  motorDBias = 1.0;
  spinRobot(true,0.15);
}

void loop() 
{
//  Serial.print("X: ");
//  Serial.print(x);
//  Serial.print("  Y: ");
//  Serial.println(y);
  
  if (checkTimeup()) {
    halt();
    next_state = HALT;
  }
  
  if (current_state != ORIENTING) {
    setBiases();
    setXY();
  }
  
  switch(current_state) {
    case HALT:
    {
      break;
    }
    case ORIENTING: {
      if (checkOriented()) {
//        driveToFirstBucket();
//        next_state = DRIVING;
        startReturning();
        next_state = RETURNING;
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
        stopDumping();
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
  // Need to call mini servo code
  miniServoUp(); // Don't have Servo code, but they should be called here
  // SET a timer...  checkDoneDumping checks for expired. How do we have timers called / configured?
}

void stopDumping() {
  miniServoDown();
}

void miniServoUp() {
  
}

void miniServoDown() {
  
}

void driveToBucket() {
  bool* vals = readFrontIR();
  int target;
  if (vals[2]) {
    target = 3;
  } else if (vals[3]) {
    target = 4;
  } else if (vals[1]) {
    target = 2;
  } else if (vals[4]) {
    target = 5;
  } else if (vals[0]) {
    target = 1;
  } else {
    target = random(1,6);
  }

  switch (target) {
    case 1: {
      driveAngle(getDestAngle(BUCKET1_X, MAX_Y),1);
      break;
    } case 2: {
      driveAngle(getDestAngle(BUCKET2_X, MAX_Y),1);
      break;
    } case 3: {
      driveAngle(getDestAngle(BUCKET3_X, MAX_Y),1);
      break;
    } case 4: {
      driveAngle(getDestAngle(BUCKET4_X, MAX_Y),1);
      break;
    } case 5: {
      driveAngle(getDestAngle(BUCKET5_X, MAX_Y),1);
      break;
    };
  }
}

void driveToFirstBucket() {
  stopDriving();
  
//  driveAngle(-90, 1);
//  driveAngle(getDestAngle(BUCKET5_X, MAX_Y),1);
}

void startReturning() {
  driveAngle(getDestAngle(0,MIN_Y),1); 
}

bool checkTape() {
  return false;
}

bool checkLoaded() {
  return false;
}

bool checkDoneReturning() {
  if (abs(x) < 3 && abs(y-MIN_Y) < 3) return true;
  else return false;
}

bool checkDoneDumping() {
  return false;
}

bool checkTimeup() {
  return false;
}


// calculates angle we need to drive at to get to a destination
int getDestAngle(int dest_x, int dest_y) {
  if (dest_y > y) {
    return 180*atan((dest_x-x)/(dest_y-y))/PI;
  } else {
    return 180+180*atan((dest_x-x)/(dest_y-y))/PI;
  }
}

// returns array of booleans for each of the 5 front IR sensors, from left to right
bool* readFrontIR() {
  bool vals[5] = {false, false, false, false, false};
  if (digitalRead(IR_PIN0) == HIGH) vals[0] = true;
  if (digitalRead(IR_PIN1) == HIGH) vals[1] = true;
  if (digitalRead(IR_PIN2) == HIGH) vals[2] = true;
  if (digitalRead(IR_PIN3) == HIGH) vals[3] = true;
  if (digitalRead(IR_PIN4) == HIGH) vals[4] = true;
  return vals;
}

bool readBackIR() {
  if (digitalRead(IR_PIN5) == HIGH) return true;
  else return false;
}

bool checkOriented() {  
  
  if (usonicValues[USONIC_RIGHTANGLE_IDX] > 0 && usonicValues[USONIC_RIGHTANGLE_IDX] > -3
      && usonicValues[USONIC_BACKANGLE_IDX] > 0 && usonicValues[USONIC_BACKANGLE_IDX] > -3
      && usonicValues[USONIC_RIGHT1_IDX] < START_RIGHTDISTANCE_MAX
      && usonicValues[USONIC_RIGHT2_IDX] < START_RIGHTDISTANCE_MAX
      && usonicValues[USONIC_BACK1_IDX] < START_BACKDISTANCE_MAX
      && usonicValues[USONIC_BACK1_IDX] < START_BACKDISTANCE_MAX
      && usonicValues[USONIC_BACK2_IDX] < START_BACKDISTANCE_MAX) {
    setXY();
    return true;
  }
  return false;
}

void setBiases() {
  int angle = 0;
  if (abs(x) > ROBOT_LENGTH+IR_BLOCK_MARGIN) {
    angle = usonicValues[USONIC_BACKANGLE_IDX];
  } else {
    angle = usonicValues[USONIC_RIGHTANGLE_IDX];
  }

  // NEEDS CALIBRATION
  motorABias = 1+angle/20.0;
  motorBBias = 1+angle/20.0;
  motorCBias = 1+angle/20.0;
  motorDBias = 1+angle/20.0;
}

void setXY() {
  int rightdistance = usonicValues[USONIC_RIGHT1_IDX];
  if (usonicValues[USONIC_RIGHT1_IDX] != -1) {
    if (usonicValues[USONIC_RIGHT2_IDX] != -1) {
      rightdistance = (usonicValues[USONIC_RIGHT1_IDX] + usonicValues[USONIC_RIGHT2_IDX])/2.0;
    } else {
      rightdistance = usonicValues[USONIC_RIGHT1_IDX];
    }
  } else {
    if (usonicValues[USONIC_RIGHT2_IDX] != -1) {
      rightdistance = usonicValues[USONIC_RIGHT2_IDX];
    } else {
      rightdistance = -1;
    }
  }
  int leftdistance = usonicValues[USONIC_LEFT_IDX];
  
  if (rightdistance != -1) {
    if (leftdistance != -1) {
      x = (leftdistance-rightdistance)/2.0;
    } else {
      x = ARENA_HALFWIDTH-rightdistance-ROBOT_LENGTH/2;
    }
  } else if (leftdistance != -1) {
      x = leftdistance-ARENA_HALFWIDTH+ROBOT_LENGTH/2;
  }
  
  if (usonicValues[USONIC_BACK1_IDX] > usonicValues[USONIC_BACK2_IDX] && usonicValues[USONIC_BACK1_IDX] != -1) {
    y = usonicValues[USONIC_BACK1_IDX];
  } else if (usonicValues[USONIC_BACK2_IDX] != -1) {
    y = usonicValues[USONIC_BACK2_IDX];
  }
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
      int speedMapping = MIN_SPEED_A + motorABias*abs(val)*(MAX_SPEED-MIN_SPEED_A);
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
      int speedMapping = MIN_SPEED_B + motorBBias*abs(val)*(MAX_SPEED-MIN_SPEED_B);
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
      int speedMapping = MIN_SPEED_C + motorCBias*abs(val)*(MAX_SPEED-MIN_SPEED_C);
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
      int speedMapping = MIN_SPEED_D + motorDBias*abs(val)*(MAX_SPEED-MIN_SPEED_D);
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


// SERIAL FUNCTIONS

// updates ultrasonic array with results
void parseString(char* result) {  
  int count = 0;
  char* token = strtok(result," ");
  while (token != NULL && count < NUM_SERIAL_TOKENS) {
    usonicValues[count] = atoi(token);
    token = strtok(NULL, " ");
    count++;
  }  
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
      break;
    case '\r':   // discard carriage return
      break;
    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_SERIAL_LEN - 1))
        input_line [input_pos++] = inByte;
      break;
  }
}


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  char* result;
  while (0 < Wire.available()) { // loop through all but the last
    result = processIncomingByte((char)Wire.read()); // receive byte as a character
  }
  Serial.println(result);
  parseString(result);
  
}
