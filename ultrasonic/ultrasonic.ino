#define ECHO_PIN1 3 // Echo Pin
#define ECHO_PIN2 5 // Echo Pin
#define ECHO_PIN3 7 // Echo Pin
#define ECHO_PIN4 9 // Echo Pin

#define TRIG_PIN1 4 // Trigger Pin
#define TRIG_PIN2 6 // Trigger Pin
#define TRIG_PIN3 8 // Trigger Pin
#define TRIG_PIN4 10 // Trigger Pin

#define MAXIMIUM_RANGE 280 // Maximum range needed (cm)
#define MAXIMUM_DURATION 16296
#define MINIMUM_RANGE 0 // Minimum range needed

#define SENSOR_SEPARATION 15.0 //cm


void setup() {
  Serial.begin (9600);
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(TRIG_PIN3, OUTPUT);
  pinMode(TRIG_PIN4, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(ECHO_PIN3, INPUT);
  pinMode(ECHO_PIN4, INPUT);
}

void loop() {
  float distance1 = pulsePin(ECHO_PIN1, TRIG_PIN1);
  float distance2 = pulsePin(ECHO_PIN2, TRIG_PIN2);
  float distance3 = pulsePin(ECHO_PIN3, TRIG_PIN3);
  float distance4 = pulsePin(ECHO_PIN4, TRIG_PIN4);

//    Serial.print(distance1);
//    Serial.print("  ");
//    Serial.print(distance2);
//    Serial.print("  ");
//    Serial.print(distance3);
//    Serial.print("  ");
//    Serial.println(distance4);

//  Serial.print(distance3);
//  Serial.print(" ");
//  Serial.print(distance4);
//  Serial.print(" ");
  Serial.print(getAngle(distance3, distance4));
  Serial.println("°");

  //Delay before next reading.
  delay(500);
}

int getAngle(float distance1, float distance2) {
  return atan((distance1 - distance2) / SENSOR_SEPARATION) * 180 / M_PI;
}

float pulsePin(int echoPin, int trigPin) {
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
  float distance = duration / 58.2;

  if (distance >= MAXIMIUM_RANGE || duration <= 0) {
    distance = -1;
  }
  return distance;


}
