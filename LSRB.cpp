#include <Arduino.h>

//  IR  pins
const int leftMostSensorPin =A4;
const int leftSensorPin = A3;
const int centerSensorPin = A2;
const int rightSensorPin = A1;
const int rightMostSensorPin = A0;

//  motor pins
const int leftMotorPin1 = 2;
const int leftMotorPin2 = 3;
const int rightMotorPin1 = 4;
const int rightMotorPin2 = 5;
#define eRightMotor 9 // right motor enable pin EA
#define eLeftMotor 10 // left motor enable pin EA
int leftMotorSpeed = 140; //speed of motor of left
int rightMotorSpeed = 140; // speed of motor of right 


// Sensor threshold values 
const int sensorThreshold = 900; // nkhyrouha 3la hssab our place  nhar competition



void setup() {
  // Motor pins setup
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  //ir pin 
     pinMode(leftSensorPin, INPUT);
  pinMode(centerSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
    pinMode(rightMostSensorPin, INPUT);
  pinMode(leftMostSensorPin, INPUT);
  


  // Serial communication for debugging
  Serial.begin(9600);


}

void loop() {
    // Read sensor values
  int leftMostSensorValue = analogRead(leftMostSensorPin);
  int leftSensorValue = analogRead(leftSensorPin);
  int centerSensorValue = analogRead(centerSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);
  int rightMostSensorValue = analogRead(rightMostSensorPin);
  


  // Determine the line following logic
  bool isLeftMostOnLine =(leftMostSensorValue > sensorThreshold);
  bool isLeftOnLine = (leftSensorValue > sensorThreshold);
  bool isCenterOnLine = (centerSensorValue > sensorThreshold);
  bool isRightOnLine = (rightSensorValue > sensorThreshold);
  bool isRightMostOnLine =(rightMostSensorValue > sensorThreshold);
  

   // left function start
  if( 
       isLeftMostOnLine or  isLeftOnLine )
              {
        turnLeft();
       
       if(isRightOnLine  ) {
        turnLeft();
          
          

         if(isCenterOnLine)
         { // it will stop turning until the central sensor detects black (90°)
           
           
          moveForward(); 
         } 
       }
       }
 
  // else
  //   {
  //     moveForward();
  //   }
     //end start function
     //forward
 if(isCenterOnLine ){
    moveForward(); 
  }
  //end forward
  //right start
 if(isRightOnLine or isRightMostOnLine )
   {
    turnRight();
   if(isLeftOnLine  ) {
        turnRight();
          
        
         if(isCenterOnLine)
         { // it will stop turning until the central sensor detects black (90°)
           moveForward(); 
         } 
       }
   }
   //right end 
  else
    {
      moveForward();
    }
  }
  



//functions 
void moveForward() {
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