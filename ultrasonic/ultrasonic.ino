#include <Wire.h>

#define ECHO_PIN1 3 // Echo Pin
#define ECHO_PIN2 7 // Echo Pin
#define ECHO_PIN3 9 // Echo Pin
#define ECHO_PIN4 11 // Echo Pin
#define ECHO_PIN5 5 // Echo Pin

#define TRIG_PIN1 4 // Trigger Pin
#define TRIG_PIN2 8 // Trigger Pin
#define TRIG_PIN3 10 // Trigger Pin
#define TRIG_PIN4 12 // Trigger Pin
#define TRIG_PIN5 6 // Trigger Pin

#define MAXIMIUM_RANGE 244 // Maximum range needed (cm)
#define MAXIMUM_DURATION 14210 // maximum µs to wait for return pulse (based on max range)
#define MINIMUM_RANGE 0 // Minimum range needed

#define SENSOR_SEPARATION 20.0 //cm
#define BAUD_RATE 115200


void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600); // serial for debugging
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(TRIG_PIN3, OUTPUT);
  pinMode(TRIG_PIN4, OUTPUT);
  pinMode(TRIG_PIN5, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(ECHO_PIN3, INPUT);
  pinMode(ECHO_PIN4, INPUT);
  pinMode(ECHO_PIN5, INPUT);
}

void loop() {
  int distance1 = pulsePin(ECHO_PIN1, TRIG_PIN1);
  int distance2 = pulsePin(ECHO_PIN2, TRIG_PIN2);
  int distance3 = pulsePin(ECHO_PIN3, TRIG_PIN3);
  int distance4 = pulsePin(ECHO_PIN4, TRIG_PIN4);
  int distance5 = pulsePin(ECHO_PIN5, TRIG_PIN5);


//  Serial.print(distance1);
//  Serial.print(" ");
//  Serial.print(distance2);
//  Serial.print(" ");
//  Serial.print(distance3);
//  Serial.print(" ");
//  Serial.print(distance4);
//  Serial.print(" ");
//  Serial.print(distance5);
//  Serial.print(" ");
//  Serial.print(getAngle(distance1, distance2));
//  Serial.print(" ");
//  Serial.println(getAngle(distance3, distance4));

  Wire.beginTransmission(8); // transmit to device #8
//  char buf1[5];
//  char buf2[5];
//  char buf3[5];
//  char buf4[5];
//  char buf5[5];
//  char buf6[5];
//  char buf7[5];
//
//  itoa(distance1,buf1,10);
//  itoa(distance2,buf2,10);
//  itoa(distance3,buf3,10);
//  itoa(distance4,buf4,10);
//  itoa(distance5,buf5,10);
//  itoa(getAngle(distance1, distance2),buf6,10);
//  itoa(getAngle(distance3, distance4),buf7,10);

  
  
//  Serial.print(buf1);
//  Serial.print(" ");
//  Serial.print(buf2);
//  Serial.print(" ");
//  Serial.println(buf6);
//  
//  
//  Wire.write(buf1);
//  Wire.write(" ");
//  Wire.write(buf2);
//  Wire.write(" ");
//  Wire.write(buf3);
//  Wire.write(" ");
//  Wire.write(buf4);
//  Wire.write(" ");
//  Wire.write(buf5);
//  Wire.write(" ");
//  Wire.write(buf6);
//  Wire.write(" ");
//  Wire.write(buf7);
//  Wire.write("\n");

  char buffer[30];
  sprintf(buffer, "%d %d %d %d %d %d %d\n", distance1, distance2, distance3, distance4, distance5, getAngle(distance1, distance2),getAngle(distance3, distance4)); 
  Serial.println(buffer);
  Wire.write(buffer);
  Wire.endTransmission();    // stop transmitting
  
}

int getAngle(float distance1, float distance2) {
  return atan((distance1 - distance2) / SENSOR_SEPARATION) * 180 / M_PI;
}

int pulsePin(int echoPin, int trigPin) {
  delayMicroseconds(MAXIMUM_DURATION);

  /* The following TRIG_PIN/echoPin cycle is used to determine the
    distance of the nearest object by bouncing soundwaves off of it. */
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  //Calculate the distance (in cm) based on the speed of sound.
  long duration = pulseIn(echoPin, HIGH, MAXIMUM_DURATION);
  int distance = duration / 58.2;

  if (distance >= MAXIMIUM_RANGE || duration <= 0) {
    distance = -1;
  }
  return distance;


}
