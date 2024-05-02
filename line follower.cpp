#include <Arduino.h>

// Definitionn of  pins of IR
const int leftSensorPin = A3;
const int centerSensorPin = A2;
const int rightSensorPin = A1;

int leftMotorSpeed = 120;
int rightMotorSpeed = 120;
int dt = 3000;
#define eRightMotor 9 // right motor enable pin EA
#define eLeftMotor 10 // left motor enable pin EA

// Definition pins of motor 
const int leftMotorPin1 = 2;
const int leftMotorPin2 = 3;
const int rightMotorPin1 = 4;
const int rightMotorPin2 = 5;

// Sensor threshold values (
const int sensorThreshold = 900 ; // nkhyroha  7na nhar competition 3la jalt  black

// functions

void goForward() {
  analogWrite(eRightMotor, rightMotorSpeed -30); // we choose a value from 0 to 255 
  analogWrite(eLeftMotor, leftMotorSpeed-30); // we choose a value from 0 to 255
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}

void turnLeft() {
  analogWrite(eRightMotor, rightMotorSpeed + 60); // we choose a value from 0 to 255 
  analogWrite(eLeftMotor, leftMotorSpeed ); // we choose a value from 0 to 255
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
}

void turnRight() {
  analogWrite(eRightMotor, rightMotorSpeed ); // we choose a value from 0 to 255 
  analogWrite(eLeftMotor, leftMotorSpeed + 60); // we choose a value from 0 to 255
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, 1);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}

void stopMotors() {
  analogWrite(eRightMotor, rightMotorSpeed); // we choose a value from 0 to 255 
  analogWrite(eLeftMotor, leftMotorSpeed); // we choose a value from 0 to 255
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
}

void setup() {
  // Motor pins setup
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(eRightMotor, OUTPUT);
  pinMode(eLeftMotor, OUTPUT);
  pinMode(leftSensorPin, INPUT);
  pinMode(centerSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
   


  
  // Serial monetor for verification
  Serial.begin(9600);
}

void loop() {
  // Read ir values
  int leftSensorValue = analogRead(leftSensorPin);
  int centerSensorValue = analogRead(centerSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);

  //output reading
  // Serial.print("Left: ");
  // Serial.print(leftSensorValue);
  // Serial.print(" Center: ");
  // Serial.print(centerSensorValue);
  // Serial.print(" Right: ");
  // Serial.println(rightSensorValue);

  // logic of working of ir
  bool isLeftOnLine = (leftSensorValue > sensorThreshold);
  bool isCenterOnLine = (centerSensorValue > sensorThreshold);
  bool isRightOnLine = (rightSensorValue > sensorThreshold);

  // Line following 
  if (isLeftOnLine && isCenterOnLine && isRightOnLine) {
    // No line detected - stop (finish) (1 st preiority )
    stopMotors();
    Serial.println("stop");
  } else if (!isLeftOnLine && isCenterOnLine && !isRightOnLine) {
    // Go straight (2nd preiority )
    goForward();
    Serial.println("forward");
  } else if (isLeftOnLine && !isCenterOnLine && !isRightOnLine) {
    // go LEFT
    turnLeft(); 
    if(isCenterOnLine){
      goForward();
    }
    Serial.println("left");
  } else if (!isLeftOnLine && !isCenterOnLine && isRightOnLine) {
    // goright
    turnRight(); 
    if(isCenterOnLine){
      goForward();
    }
   Serial.println("right");
  }
}
