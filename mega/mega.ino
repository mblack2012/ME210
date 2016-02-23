#define BAUD_RATE 9600

#define ULTRASONIC_SERIAL Serial1
#define OTHER_SERIAL Serial2

#define MOTOR1_PWM_PIN 2
#define MOTOR1_DIR_PIN 22

#define MOTOR2_PWM_PIN 3
#define MOTOR2_DIR_PIN 23

#define MOTOR3_PWM_PIN 4
#define MOTOR3_DIR_PIN 24

#define MOTOR4_PWM_PIN 5
#define MOTOR4_DIR_PIN 25

#define MAX_INPUT 4

void setup() {
  ULTRASONIC_SERIAL.begin(BAUD_RATE);
  OTHER_SERIAL.begin(BAUD_RATE);
  Serial.begin(9600);
  
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_PWM_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR3_PWM_PIN, OUTPUT);
  pinMode(MOTOR3_DIR_PIN, OUTPUT);
  pinMode(MOTOR4_PWM_PIN, OUTPUT);
  pinMode(MOTOR4_DIR_PIN, OUTPUT);
}

void process_data (const char * data){
  Serial.println((String)data);
}

char* processIncomingByte (const byte inByte) {
  static char input_line [MAX_INPUT];
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
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;
  }
  return "";
}

int readUltrasonicSerial() {
  static int angle = 0;
  
  if (!ULTRASONIC_SERIAL.available()) return angle;
  
  const char* ultrasonicResult;
  // once all bytes have been read, result will contain the resulting string
  while (ULTRASONIC_SERIAL.available()) {
    ultrasonicResult = processIncomingByte(ULTRASONIC_SERIAL.read ());
  }
  ULTRASONIC_SERIAL.flush();
  if (sizeof(ultrasonicResult) != 0) {
    angle = atoi(ultrasonicResult);
  }
  
  return angle; 
}

void 

void loop() {
  int angle = readUltrasonicSerial();
  
}
