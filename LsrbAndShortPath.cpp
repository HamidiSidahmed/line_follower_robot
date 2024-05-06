
#include <Arduino.h>
// IR sensor pins
const int leftMostSensorPin = A2;
const int leftSensorPin = A1;
const int centerSensorPin = A0;
const int rightSensorPin = A3;
const int rightMostSensorPin = A4;

// Motor pins
const int leftMotorPin1 = 5;
const int leftMotorPin2 = 4;
const int rightMotorPin1 = 2;
const int rightMotorPin2 = 3;
#define eRightMotor 10 // Right motor enable pin EA
#define eLeftMotor 11  // Left motor enable pin EA

// Speed of motors
int leftMotorSpeed = 140;  // Speed of the left motor
int rightMotorSpeed = 140; // Speed of the right motor

// Sensor threshold value
const int sensorThreshold = 900; // Threshold for detecting a line

// Short path variables
const int maxPathSize = 100; // Maximum size  array
char path[maxPathSize];       // Array to record the path
int stepsNumber = 0;           // Counter for the steps number
char lastDirection = ' ';      // Variable to store the last direction
char opt[maxPathSize]; // an array to remove spacices after optimizing


//FUNCTION OF STORAGE 
void storeDirection(char direction)
{
    // Check if the present direction is different from the last direction
    if (direction != lastDirection)
    {
        // Store the direction in the path array
        path[stepsNumber] = direction;
        // Increment the nbr step
        stepsNumber++;
        // Update the last direction
        lastDirection = direction;
    }
}
// Function to summarize the path 
void summarizePath(char* path, int& stepsNumber)
{
    // Loop through the path array
    start: for(int i = 0; i < stepsNumber; i++) {
     if (path[i] == 'L' && path[i + 1] == 'B' && path[i + 2] == 'R') //lbr=b
        {
            path[i] = 'B';
            path[i + 1] = ' ';
            path[i + 2] = ' ';
        }
  else if (path[i] == 'L' && path[i + 1] == 'B' && path[i + 2] == 'S')//lbs=r
        {
            path[i] = 'R';
            path[i + 1] = ' ';
            path[i + 2] = ' ';
        }
  else if (path[i] == 'R' && path[i + 1] == 'B' && path[i + 2] == 'L')//rbl=b
        {
            path[i] = 'B';
            path[i + 1] = ' ';
            path[i + 2] = ' ';
        }
  else if (path[i] == 'S' && path[i + 1] == 'B' && path[i + 2] == 'L') //sbl=r
        {
            path[i] = 'R';
            path[i + 1] = ' ';
            path[i + 2] = ' ';
        }
   else if (path[i] == 'S' && path[i + 1] == 'B' && path[i + 2] == 'S') //sbs=b
        {
            path[i] = 'B';
            path[i + 1] = ' ';
            path[i + 2] = ' ';
        }
   else if (path[i] == 'L' && path[i + 1] == 'B' && path[i + 2] == 'L') //lbl=s
        {
            path[i] = 'S';
            path[i + 1] = ' ';
            path[i + 2] = ' ';
        }
    }
    
    //shifttt  to remove spaces
    int shiftCount = 0; 
    for( int i = 0; i < stepsNumber; i++)
    {
       if (path[i]==' ')
        {
          shiftCount++;
        }
       else 
        {
         opt[i-shiftCount]=path[i];
        }
      }
       for( int i = 0; i < stepsNumber; i++){
       path[i]=opt[i];}
       
      for( int i = 0; i < stepsNumber; i++){
      if (path[i]=='B'){
      goto start;
      }
      }
      
}
// Functions for motor dir
void moveForward() {
  analogWrite(eRightMotor, rightMotorSpeed - 30); // Adjusted speed
  analogWrite(eLeftMotor, leftMotorSpeed - 30); // Adjusted speed
  digitalWrite(leftMotorPin1, HIGH);

digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}

void turnLeft() {
  analogWrite(eRightMotor, rightMotorSpeed + 60); // Adjusted speed
  analogWrite(eLeftMotor, leftMotorSpeed); // No adjustment
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}


void turnRight() {
  analogWrite(eRightMotor, rightMotorSpeed); // No adjustment
  analogWrite(eLeftMotor, leftMotorSpeed + 60); // Adjusted speed
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
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
  // Setup motor pins
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  // Setup IR sensor pins
  pinMode(leftSensorPin, INPUT);
  pinMode(centerSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  pinMode(rightMostSensorPin, INPUT);
  pinMode(leftMostSensorPin, INPUT);

  // Setup serial communication for debugging
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
  bool isLeftMostOnLine = (leftMostSensorValue > sensorThreshold);
  bool isLeftOnLine = (leftSensorValue > sensorThreshold);
  bool isCenterOnLine = (centerSensorValue > sensorThreshold);
  bool isRightOnLine = (rightSensorValue > sensorThreshold);
  bool isRightMostOnLine = (rightMostSensorValue > sensorThreshold);
  
  // Line following logic

// Check if all sensors detect the black line 
  if (isLeftMostOnLine && isLeftOnLine && isCenterOnLine && isRightOnLine && isRightMostOnLine) {
     void stopMotors();

    // Summarize the path
    summarizePath(path, stepsNumber);
    
    // Reset the stepsNumber to the length of the summarized path
    stepsNumber = strlen(path);
       if ( isCenterOnLine ){
    // Move based on the shortened path
    for (int i = 0; i < stepsNumber; i++) {
      if (path[i] == 'L'&& (isLeftMostOnLine) ) { 
        turnLeft();
          if (isRightMostOnLine) {
      turnLeft();

      if (isCenterOnLine) {
        // Move forward if center sensor detects a line (90° turn complete)
        moveForward();  
      } 
    }
      }
       else if (path[i] == 'R' && ( isRightMostOnLine)) {
        turnRight();
        if (isLeftOnLine) {
          // Adjust right turn if left sensor detects a line
          turnRight();
          if (isCenterOnLine) {
            // Move forward if center sensor detects a line (90° turn complete)
            moveForward(); 
          } 
        }
      }
       else if (path[i] == 'S' && isCenterOnLine) {
        moveForward();
      }
      delay(500); // Adjust the delay according to your requirements
    }
  }
  }
if (isLeftMostOnLine || isLeftOnLine) {
    // Left turn logic
    turnLeft();

    if (isRightMostOnLine) {
      // Adjust left turn if right sensor detects a line
      turnLeft();

      if (isCenterOnLine) {
        // Move forward if center sensor detects a line (90° turn complete)
        moveForward();  
      } 
    }
    // Store the direction in the path array as 'L'
    if (lastDirection != 'L')
        {
            storeDirection('L');
        }
  }
  else {
    // Center (straight) logic
    if (isCenterOnLine) {
      // Move forward logic
      moveForward();  
      //straight logic
        if (lastDirection != 'S')
        {
            storeDirection('S');
        }
    }
    else {
      // Right turn logic
      if (isRightOnLine || isRightMostOnLine) {
        turnRight();
        if (isLeftOnLine) {
          // Adjust right turn if left sensor detects a line
          turnRight();
          if (isCenterOnLine) {
            // Move forward if center sensor detects a line (90° turn complete)
            moveForward(); 
          } 
        }
         // Right turn logic
        if (lastDirection != 'R')
        {
            storeDirection('R');
        }
      }
      else {
        // Backward logic
        if (!isLeftMostOnLine && !isLeftOnLine && !isCenterOnLine && !isRightOnLine && !isRightMostOnLine) {
          // Turn 180° if all sensors detect white
          turnRight(); 
          if (isCenterOnLine) {
            // Move forward if center sensor detects a line (180° turn complete)
            moveForward(); 
          } 
           // Center (straight) logic
        if (lastDirection != 'B')
        {
            storeDirection('B');
        }
        }
      }
    }
  } 
}