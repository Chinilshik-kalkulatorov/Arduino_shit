#include <AFMotor.h>  
#include <NewPing.h> 
#include <Servo.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define TRIG_PIN A0 
#define ECHO_PIN A1
#define MAX_DISTANCE 200
#define MAX_SPEED 250
#define IR_PIN A2
#define IR_PIN2 A3
#define BT_RX 0
#define BT_TX 1
#define LCD_SDA A4
#define LCD_SCL A5

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 
AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);
Servo myservo;
LiquidCrystal_I2C lcd(0x27, 16, 2); 
SoftwareSerial BTSerial(BT_TX, BT_RX);

bool isAutomatic = true;
boolean goesForward = false;
int ultrasonicDistance = 100;
int speedSet = 0;

int lastUltrasonicDistance = -1;
int lastIrDistance = -1;
int lastIrDistance2 = -1;

void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, INPUT);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  myservo.attach(10);
  myservo.write(110);
  delay(1000);
  ultrasonicDistance = readPing();
  delay(100);
  ultrasonicDistance = readPing();
  delay(100);
  ultrasonicDistance = readPing();
  delay(100);
  ultrasonicDistance = readPing();
  delay(100);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'S') {
      isAutomatic = !isAutomatic;
      moveStop();
      delay(500);
    } else if (!isAutomatic) {
      switch (command) {
      case 'F': forward(); break;
      case 'B': back(); break;
      case 'L': left(); break;
      case 'R': right(); break;
      default: moveStop(); break;
      }
    }
  }

  if (isAutomatic) {
    autoAvoidance();
  }
}

void autoAvoidance() {
  int distanceR = 0;
  int distanceL = 0;
  int irDistance = digitalRead(IR_PIN);
  int irDistance2 = digitalRead(IR_PIN2);
  ultrasonicDistance = readPing();

  delay(40);

  if (getIRStatus()== LOW|| getIR2Status()== LOW) {
    moveStop();
    delay(100);
    moveBackward();
    delay(300);
    moveStop();
    delay(200);
    distanceR = lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);
    if (distanceR >= distanceL) {
      turnRight();
    } else {
      turnLeft();
    }
    moveStop();
  } else if (ultrasonicDistance <= 30) {
    moveStop();
    delay(100);
    moveBackward();
    delay(300);
    moveStop();
    delay(200);
    distanceR = lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);

    if (distanceR >= distanceL) {
      turnRight();
      moveStop();
    } else {
      turnLeft();
      moveStop();
    }
  } else {
    moveForward();
  }
  ultrasonicDistance = readPing();
}

int lookRight() {
  myservo.write(50);
  delay(500);
  int ultrasonicDistance = readPing();
  delay(100);
  myservo.write(115);
  return ultrasonicDistance;
}

int lookLeft() {
  myservo.write(170);
  delay(500);
  int ultrasonicDistance = readPing();
  delay(100);
  myservo.write(115);
  return ultrasonicDistance;
}

int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
}

void moveStop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
} 

void moveForward() {
  if (!goesForward) {
    goesForward = true;
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2)
    {
      motor1.setSpeed(speedSet);
      motor2.setSpeed(speedSet);
      motor3.setSpeed(speedSet);
      motor4.setSpeed(speedSet);
      delay(5);
    }
  }
}

void moveBackward() {
  goesForward = false;
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2)
  {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}  

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(500);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
} 

void turnLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(500);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}
/*------------Manual part-------------*/
 void forward()
{
  motor1.setSpeed(255);  
  motor1.run(FORWARD);  
  motor2.setSpeed(255);  
  motor2.run(FORWARD);  
  motor3.setSpeed(255); 
  motor3.run(FORWARD);  
  motor4.setSpeed(255); 
  motor4.run(FORWARD);  
}

void back()
{
  motor1.setSpeed(255);  
  motor1.run(BACKWARD);  
  motor2.setSpeed(255);  
  motor2.run(BACKWARD);  
  motor3.setSpeed(255);  
  motor3.run(BACKWARD);  
  motor4.setSpeed(255);  
  motor4.run(BACKWARD);  
}

void left()
{
  motor1.setSpeed(255);  
  motor1.run(BACKWARD);  
  motor2.setSpeed(255);  
  motor2.run(BACKWARD);  
  motor3.setSpeed(255);  
  motor3.run(FORWARD);   
  motor4.setSpeed(255);  
  motor4.run(FORWARD);   
}

void right()
{
  motor1.setSpeed(255); 
  motor1.run(FORWARD);  
  motor2.setSpeed(255);  
  motor2.run(FORWARD);  
  motor3.setSpeed(255);  
  motor3.run(BACKWARD);  
  motor4.setSpeed(255);  
  motor4.run(BACKWARD);  
} 

void Stop()
{
  motor1.setSpeed(0);  
  motor1.run(RELEASE);  
  motor2.setSpeed(0);  
  motor2.run(RELEASE);  
  motor3.setSpeed(0);  
  motor3.run(RELEASE);  
  motor4.setSpeed(0);  
  motor4.run(RELEASE);  
}

bool getIRStatus() {
  return digitalRead(IR_PIN) == LOW;
}

bool getIR2Status() {
  return digitalRead(IR_PIN2) == LOW;
}
