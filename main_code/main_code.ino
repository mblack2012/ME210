#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

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
#define TAPE_PIN 24

//Gate Servo 1
#define GATE_SERVO 4
Servo gateServo;

//Gate Servo 0
#define GATE_SERVO_0 13
Servo gateServo_0;

//Ramp Servo
#define RAMP_SERVO 5
Servo rampServo;

//motor A
#define PWM_PIN_A 9  // pwm pin for the motor
#define DIR_PIN_A1 51 //direction input to the H bridge
#define DIR_PIN_A2 53 //direction input to the H bridge


//motor B
#define PWM_PIN_B 11  // pwm pin for the motor
#define DIR_PIN_B1 47 //direction input to the H bridge
#define DIR_PIN_B2 49 //direction input to the H bridge

//motor C
#define PWM_PIN_C 6  // pwm pin for the motor
#define DIR_PIN_C 7 //direction input to the H bridge

//motor D
#define PWM_PIN_D 3  // pwm pin for the motor
#define DIR_PIN_D 2 //direction input to the H bridge

#define MIN_SPEED_A 85.0
#define MIN_SPEED_B 85.0
#define MIN_SPEED_C 85.0
#define MIN_SPEED_D 85.0
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

#define SENSOR_SEPARATION 20.0 //cm

#define START_RIGHTDISTANCE_MAX 55 // cm
#define START_BACKDISTANCE_MAX 55 // cm

#define ARENA_HALFWIDTH 122 // cm
#define ARENA_WIDTH 244
#define ROBOT_LENGTH 28 // cm

// half width of the back IR block
#define IR_BLOCK_MARGIN 8 // cm

// assumed y thickness of the IR block
#define MIN_Y 8 // cm

// assumed Y coord of back of robot when we dump
#define MAX_Y 68 // cm

// coordinates of the centers of the buckets, from left to right
#define BUCKET1_X -76 // cm (-32in)
#define BUCKET2_X -58 // cm (-16in)
#define BUCKET3_X  -0 // cm (0in)
#define BUCKET4_X  58 // cm (16in)
#define BUCKET5_X  76  // cm (32in)

//keep track of ramp deployment
bool deployed = false;

/*---------------Module Function Prototypes---*/
unsigned char TestForKey(void);
void RespToKey(void);
void spinRobot(bool direction, double speed);
void driveGateServo();
void driveGate0Servo();
void deployRampServo(Servo rampServo, bool* deployed);
void startDumping();
void startDumping0();
void deployChips();

// FSM STATE DEFINITIONS
typedef enum{
  HALT, //stops the robot
  INITIAL_ORIENTING, //spin and orient the robot and determine the initial x,y coordinates
  ORIENTING,
  DRIVING, //drives the robot 
  RETURNING, //return the robot to the base
  DUMPING, //dump chips
  LOADING //the robot waits until the chips are loaded
} states;

/*----------------Global variables------------*/
states current_state;
states next_state;

//number of chips deployed so far
int n_chips_deployed = 0;
int target;

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
  pinMode(DIR_PIN_A1, OUTPUT); 
  pinMode(DIR_PIN_A2, OUTPUT); 
  pinMode(PWM_PIN_B, OUTPUT);
  pinMode(DIR_PIN_B1, OUTPUT);
  pinMode(DIR_PIN_B2, OUTPUT); 
  pinMode(PWM_PIN_C, OUTPUT);
  pinMode(DIR_PIN_C, OUTPUT); 
  pinMode(PWM_PIN_D, OUTPUT);
  pinMode(DIR_PIN_D, OUTPUT);
  pinMode(TAPE_PIN, INPUT);
  
  //digitalWrite(PWM_PIN_A,LOW);
  //digitalWrite(PWM_PIN_B,LOW);
  //digitalWrite(PWM_PIN_C,LOW);
  //digitalWrite(PWM_PIN_D,LOW);
  
  //Begin from the clockwise spinning state
  //current_state = INITIAL_ORIENTING;
  //next_state = INITIAL_ORIENTING;
  stopDriving();
  
  delay(300);

  x = 90; // initial guess
  y = 20; // initial guess

  motorABias = 1.0;
  motorBBias = 1.0;
  motorCBias = 1.0;
  motorDBias = 1.0;

  
//  Set up the servos
  gateServo.attach(GATE_SERVO);
  gateServo.write(170);
  gateServo_0.attach(GATE_SERVO_0);
  gateServo_0.write(0);


  
  spinRobot(true,0.15);
  current_state = INITIAL_ORIENTING;
  next_state = INITIAL_ORIENTING; 

//  driveAngle(0,0.5);
//  current_state = DRIVING;
//  next_state = DRIVING;
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
  
  if (current_state != INITIAL_ORIENTING && current_state != ORIENTING) {
//    setBiases();
    setXY();
  }
  
  switch(current_state) {
    case HALT:
    {
      //Serial.print(checkTape());
      //deployRampServo(rampServo, &deployed);
      //driveGateServo(gateServo);
//      Serial.println("HALT");
      next_state = HALT;
      break;
    }
    case INITIAL_ORIENTING: {
      if (checkOriented()) {
        stopDriving();

        delay(50);
        precisionOrient(true);
        
        target = 5;
        driveToFirstBucket();
        next_state = DRIVING;
      }
      break;
    }
    case ORIENTING: { 
      // resetBiases before this state
      if (checkOriented2()) {
        if (y > ARENA_WIDTH/4) {
          startReturning();
          next_state = RETURNING;
        } else {
          driveToBucket();
          next_state = DRIVING;
        }
      }
    }
    break;
    case DRIVING: {
//      if (checkTape()) {
//        stopDriving();
//        deployChips();
//        next_state = DUMPING;
//      }
      if (target == 5) {
        if (checkTape()) {
          stopDriving();
          precisionOrient(false);
          driveAngle(0,0.25);
          delay(80);
          stopDriving();
          deployChips();
          delay(500);
          driveAngle(180,0.25);
          delay(100);
          stopDriving();
          precisionOrient(false);
          target--;
        }
        
      } else if (target == 4) {
        driveAngle(-95,0.5);
        delay(1300);
        stopDriving();
        driveAngle(0,0.25);
        delay(100);
        stopDriving();
        delay(300);
        deployChips();
        target--;
        
      } else if (target == 3) {
        driveAngle(-95,0.5);
        delay(1300);
        stopDriving();
        driveAngle(0,0.25);
        delay(200);
        stopDriving();
        
        deployChips();
        delay(200);
        driveAngle(180,0.25);
        delay(600);
        stopDriving();
//        precisionOrient(false);
        delay(100);
        
        gateServo.attach(GATE_SERVO);
        gateServo.write(170);
        gateServo_0.attach(GATE_SERVO_0);
        gateServo_0.write(0);
        
        n_chips_deployed = 1;
        startReturning();
        next_state = RETURNING;

      } else if (checkAtBucket()) {
        stopDriving();
        deployChips();
        delay(300);
//        driveToBucket();
        driveAngle (-95,0.5);
        target--;
      } else {
        stopDriving();
        delay(1000);
      }
      break;
    }
    case DUMPING: {
      if (checkDoneDumping()) {
//        stopDumping();
        startReturning();
        next_state = DRIVING;
      }
      break;
    }
    case RETURNING: {
      if (checkDoneReturning()) {
        stopDriving();
        next_state = LOADING;
      } else if (abs(x) < 5) {
        driveAngle(180,0.5);
        
      }
      break;
    } case LOADING: {
      if (checkLoaded()) {
        target = 5;
//        driveToBucket();
        driveAngle(45,0.5);
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
//  stopDumping();
}

void stopDriving() {
//  analogWrite(PWM_PIN_A, MIN_SPEED_A);
//  analogWrite(PWM_PIN_B, MIN_SPEED_B);
//  analogWrite(PWM_PIN_C, MIN_SPEED_C);
//  analogWrite(PWM_PIN_D, MIN_SPEED_D);
  digitalWrite(PWM_PIN_A, 0);
  digitalWrite(PWM_PIN_B, 0);
  digitalWrite(PWM_PIN_C, 0);
  digitalWrite(PWM_PIN_D, 0);
}

void startDumping() {
  // Need to call mini servo code
  driveGateServo();
  gateServo.detach();
  delay(1500);
}

void startDumping0() {
  // Need to call mini servo code
  driveGate0Servo();
  gateServo_0.detach();
  delay(1500);
}

void precisionOrient(bool useBack) {
  delay(100);
  if (useBack) {
    float angle1 = atan((usonicValues[1] - usonicValues[0]) / SENSOR_SEPARATION) * 180 / M_PI;
    float angle2 = atan((usonicValues[3] - usonicValues[2]) / SENSOR_SEPARATION) * 180 / M_PI;
     
    while (abs(angle1)>1 || abs(angle2)>1) {
      if (angle1 < 0) {
        spinRobot(true,0.40);
        delay(7+sqrt(abs(angle1))*20);
        stopDriving();
      } else {
        spinRobot(false,0.40);
        delay(7+sqrt(abs(angle1))*20);
        stopDriving();
      }
      delay(200);
      angle1 = atan((usonicValues[1] - usonicValues[0]) / SENSOR_SEPARATION) * 180 / M_PI;
      angle2 = atan((usonicValues[3] - usonicValues[2]) / SENSOR_SEPARATION) * 180 / M_PI;
    }
  } else {
    float angle1 = atan((usonicValues[1] - usonicValues[0]) / SENSOR_SEPARATION) * 180 / M_PI;
     
    while (abs(angle1)>1) {
      if (angle1 < 0) {
        spinRobot(true,0.40);
        delay(7+sqrt(abs(angle1))*20);
        stopDriving();
      } else {
        spinRobot(false,0.40);
        delay(7+sqrt(abs(angle1))*20);
        stopDriving();
      }
      delay(200);
      angle1 = atan((usonicValues[1] - usonicValues[0]) / SENSOR_SEPARATION) * 180 / M_PI;
    }
  }
  
  stopDriving();
  delay(500);
//  delay(100000);
}


void driveToBucket() {
  
//  bool* vals = readFrontIR();
//  int target;
//  if (vals[2]) {
//    target = 3;
//  } else if (vals[3]) {
//    target = 4;
//  } else if (vals[1]) {
//    target = 2;
//  } else if (vals[4]) {
//    target = 5;
//  } else if (vals[0]) {
//    target = 1;
//  } else {
//    target = random(1,6);
//  }

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
  driveAngle(getDestAngle(BUCKET5_X, MAX_Y-5),0.5);
}

void startReturning() {
  driveAngle(getDestAngle(0,MIN_Y),0.5); 
}

bool checkTape() {
  return !digitalRead(TAPE_PIN) || y > MAX_Y - 6;
//  return y > MAX_Y - 6;
  
//  return false;
}

bool checkAtBucket() {
  switch (target) {
    case 0: {
      stopDriving();
      delay(10000);
      break;
    }
    case 1: {
      if (abs(x-BUCKET1_X) < 5) return true;
      break;
    }
    case 2: {
      if (abs(x-BUCKET2_X) < 5) return true;
      break;
    }
    case 3: {
      if (abs(x-BUCKET3_X) < 5) return true;
      break;
    }
    case 4: {
      if (abs(x-BUCKET4_X) < 5) return true;
      break;
    }
    case 5: {
      if (abs(x-BUCKET5_X) < 5) return true;
      break;
    }
  }
  return false;
}

bool checkLoaded() {
  delay(3000);
  return true;
}

bool checkDoneReturning() {
  if (abs(y-MIN_Y) < 3) return true;
  else return false;
}

bool checkDoneDumping() {
  return true;
//  return false;
}

bool checkTimeup() {
  return false;
}


// calculates angle we need to drive at to get to a destination
int getDestAngle(int dest_x, int dest_y) {
  if (dest_y > y) {
    return 180*atan((float)(dest_x-x)/(dest_y-y))/M_PI;
  } else {
    return 180+180*atan((float)(dest_x-x)/(dest_y-y))/M_PI;
  }
}

// returns array of booleans for each of the 5 front IR sensors, from left to right
bool* readFrontIR() {
  bool vals[5] = {true, true, false, true, false};
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
  
  if (abs(usonicValues[USONIC_RIGHTANGLE_IDX]+20) < 5
      && abs(usonicValues[USONIC_BACKANGLE_IDX]+20) < 5
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

bool checkOriented2() {
  if (abs(usonicValues[USONIC_RIGHTANGLE_IDX]+12) < 5
      && abs(usonicValues[USONIC_LEFT_IDX] + ROBOT_LENGTH + usonicValues[USONIC_RIGHT1_IDX] - ARENA_WIDTH) < 5) {
        setXY();
    return true;
  } 
  return false;
}

void resetBiases() {
  motorABias = 1;
  motorBBias = 1;
  motorCBias = 1;
  motorDBias = 1;
}

void setBiases() {
  int angle = 0;
//  if (abs(x) > ROBOT_LENGTH+IR_BLOCK_MARGIN) {
//    angle = usonicValues[USONIC_BACKANGLE_IDX];
//  } else {
//    angle = usonicValues[USONIC_RIGHTANGLE_IDX];
//  }
  angle = usonicValues[USONIC_RIGHTANGLE_IDX];

  // NEEDS CALIBRATION
  motorABias = 1+angle/5.0;
  motorBBias = 1-angle/5.0;
  motorCBias = 1-angle/5.0;
  motorDBias = 1+angle/5.0;
}

void setXY() {
  int rightdistance = usonicValues[USONIC_RIGHT1_IDX];
//  rightdistance = (usonicValues[USONIC_RIGHT1_IDX] + usonicValues[USONIC_RIGHT2_IDX])/2.0;
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
//  int leftdistance = usonicValues[USONIC_LEFT_IDX];
//  
//  if (rightdistance != -1) {
//    if (leftdistance != -1) {
//      x = (leftdistance-rightdistance)/2.0;
//    } else {
//      x = ARENA_HALFWIDTH-rightdistance-ROBOT_LENGTH/2;
//    }
//  } else if (leftdistance != -1) {
//      x = leftdistance-ARENA_HALFWIDTH+ROBOT_LENGTH/2;
//  }
  x = ARENA_HALFWIDTH-rightdistance-ROBOT_LENGTH/2;
  
  
  if (usonicValues[USONIC_BACK1_IDX] > usonicValues[USONIC_BACK2_IDX] && usonicValues[USONIC_BACK2_IDX] != -1) {
    y = usonicValues[USONIC_BACK2_IDX];
  } else if (usonicValues[USONIC_BACK1_IDX] != -1) {
    y = usonicValues[USONIC_BACK1_IDX];
  }

//  if (usonicValues[USONIC_BACK1_IDX] != -1) {
//    if (usonicValues[USONIC_BACK2_IDX] != -1) {
//      y = (usonicValues[USONIC_BACK1_IDX]+usonicValues[USONIC_BACK2_IDX])/2;
//    }
//  } else if (usonicValues[USONIC_BACK2_IDX] != -1) {
//    y = usonicValues[USONIC_BACK2_IDX];
//  }

  Serial.println(x);
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
  Serial.print(motor);
  Serial.print(" ");
  Serial.println(val);
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
      if (val < 0) {
        digitalWrite(DIR_PIN_A1, LOW);
        digitalWrite(DIR_PIN_A2, HIGH);
      }
      else {
        digitalWrite(DIR_PIN_A1, HIGH);
        digitalWrite(DIR_PIN_A2, LOW);
      }
      break;
    };
    case 2: {
      if (abs(val) < 0.01) {
        digitalWrite(PWM_PIN_B, LOW); 
        break;
      }
      int speedMapping = MIN_SPEED_B + motorBBias*abs(val)*(MAX_SPEED-MIN_SPEED_B);
      analogWrite(PWM_PIN_B, speedMapping);
      if (val < 0) {
        digitalWrite(DIR_PIN_B1, HIGH);
        digitalWrite(DIR_PIN_B2, LOW);
      }
      else {
        digitalWrite(DIR_PIN_B1, LOW);
        digitalWrite(DIR_PIN_B2, HIGH);
      }
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
  usonicValues[5] = atan((usonicValues[1] - usonicValues[0]) / SENSOR_SEPARATION) * 180 / M_PI;
  usonicValues[6] = atan((usonicValues[3] - usonicValues[2]) / SENSOR_SEPARATION) * 180 / M_PI;
//
//  for (int i=0; i<7; i++) {
//    Serial.print(usonicValues[i]);
//    Serial.print(" ");
//  }
//  Serial.println();
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
//  Serial.println(result);
  parseString(result);
  
}

void driveGateServo(){
  int pos;

  for(pos = 160; pos>=35; pos-=5)     // goes from 180 degrees to 0 degrees 
  {                                
    gateServo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }

  gateServo.detach();
  /*
  delay(1000);
  CurrentgateServo.attach(GATE_SERVO); 
  
  for(pos = 35; pos <= 165; pos += 5) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    CurrentgateServo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
  */
}

void driveGate0Servo(){
  int pos;

  for(pos = 20; pos <= 90; pos += 5) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    gateServo_0.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
  gateServo_0.detach();
  
  /*
  delay(1000);
  CurrentgateServo.attach(GATE_SERVO); 
  
  for(pos = 35; pos <= 165; pos += 5) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    CurrentgateServo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  }
  */
}

void deployRampServo(Servo rampServo, bool* deployed){
    if (!(*deployed)){
      rampServo.write(1200);    // Rotate servo to center
      delay(800);
      rampServo.write(1500);     // Rotate servo clockwise
      delay(1000);
    }
    *deployed = true; 
}

void deployChips(){
  if (n_chips_deployed == 0){
    rampServo.attach(RAMP_SERVO);
    deployRampServo(rampServo, &deployed);
    rampServo.detach();
    n_chips_deployed += 1;
    return;
  }
  if (n_chips_deployed == 1){
    startDumping0();
    n_chips_deployed += 1;
    return;
  }
  if (n_chips_deployed == 2){
    startDumping();
    n_chips_deployed += 1;
    return;
  }
}
