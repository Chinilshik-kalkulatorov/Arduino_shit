#include <Servo.h>

// Define pin connections
const int motorPins[4][2] = {{4, 7}, {2, 3}};
const int enablePins[2] = {9, 10};
const int servoPin = 12, irPin = 11, trigPin = 8, echoPin = 6, buzzerPin = A1;

Servo servoMotor;
unsigned long previousMillis = 0;  // to manage timing without delay
const long interval = 300;         // interval at which to scan (milliseconds)

// Function prototypes
void drive(int speed, bool forward);
void stopMotors();
void beep(int count, int toneFreq, int duration);
void scanAndCheckObstacles();
long measureDistance();

void setup() {
  for (auto &pinPair : motorPins) {
    pinMode(pinPair[0], OUTPUT);
    pinMode(pinPair[1], OUTPUT);
  }
  for (int pin : enablePins) pinMode(pin, OUTPUT);
  pinMode(irPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  servoMotor.attach(servoPin);
  Serial.begin(9600);
  beep(3, 1000, 100);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    scanAndCheckObstacles();
  }
}

void scanAndCheckObstacles() {
  static int angle = 0;
  static bool increasing = true;

  servoMotor.write(angle);
  if (increasing) {
    angle += 30;
    if (angle >= 180) increasing = false;
  } else {
    angle -= 30;
    if (angle <= 0) increasing = true;
  }

  long distance = measureDistance();
  if (digitalRead(irPin) == LOW || distance < 20) {
    Serial.println("Obstacle detected - Stopping");
    beep(1, 2000, 500);
    stopMotors();
    drive(-150, false);
    delay(1500);
    stopMotors();
  } else {
    drive(255, true);
  }
}

long measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}

void drive(int speed, bool forward) {
  int directionMult = forward ? HIGH : LOW;
  for (int i = 0; i < 2; i++) {
    analogWrite(enablePins[i], abs(speed));
    digitalWrite(motorPins[i][0], directionMult);
    digitalWrite(motorPins[i][1], !directionMult);
  }
}

void stopMotors() {
  for (int pin : enablePins) analogWrite(pin, 0);
}

void beep(int count, int toneFreq, int duration) {
  for (int i = 0; i < count; i++) {
    tone(buzzerPin, toneFreq, duration);
    delay(duration + 100);
    noTone(buzzerPin);
  }
}
