
//  IR  pins
const int leftMostSensorPin = A2;
const int leftSensorPin = A1;
const int centerSensorPin = A0;
const int rightSensorPin = A3;
const int rightMostSensorPin = A4;

//  motor pins
const int leftMotorPin1 = 5;
const int leftMotorPin2 = 4;
const int rightMotorPin1 = 2;
const int rightMotorPin2 = 3;
#define eRightMotor 10      // right motor enable pin EA
#define eLeftMotor 11       // left motor enable pin EA
int leftMotorSpeed = 90;   //speed of motor of left
int rightMotorSpeed = 90;  // speed of motor of right


// Sensor threshold values
const int sensorThreshold = 700;  // nkhyrouha 3la hssab our place  nhar competition

// PID stuff
int sensorPin[3] = { A0, A1, A3 };

int sensorValue[3];

int currentWeight;

int baseSpeed = 110;
int kp = 95;
float ki = 0.5;
int kd =300;

int error;
int previousError = 0;
int integral = 0;
int derivative;
int idealSum = 0;

float correction;

long prevMillis = -3500;

// shortest path stuff
String path;

int leftMostSensorValue;
int leftSensorValue;
int centerSensorValue;
int rightSensorValue;
int rightMostSensorValue;

bool isLeftMostOnLine;
bool isLeftOnLine;
bool isCenterOnLine;
bool isRightOnLine;
bool isRightMostOnLine;

void setup() {
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(eRightMotor, OUTPUT);
  pinMode(eLeftMotor, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  path = path + "S";
  Serial.begin(9600);
}

void loop() {
  readSensors();
  makeDicision();
}



//functions
void moveForward() {
  analogWrite(eRightMotor, rightMotorSpeed  );  // we choose a value from 0 to 255
  analogWrite(eLeftMotor, leftMotorSpeed );    // we choose a value from 0 to 255
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}

void moveBackward() {

  analogWrite(eRightMotor, rightMotorSpeed + 70);  // we choose a value from 0 to 255
  analogWrite(eLeftMotor, leftMotorSpeed );    // we choose a value from 0 to 255
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);

  while (!isCenterOnLine && !isRightOnLine ) {
    readSensors();
  }
}

void turnRight() {
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  analogWrite(eRightMotor, 40) ;     // we choose a value from 0 to 255
  analogWrite(eLeftMotor, 180);  // we choose a value from 0 to 255
  while (!isCenterOnLine) {
    readSensors();
  }
}

void turnLeft() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, 1);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(eRightMotor, 180);  // we choose a value from 0 to 255
  analogWrite(eLeftMotor, 30);         // we choose a value from 0 to 255
  while (isCenterOnLine) {
    readSensors();
    Serial.println("stuck in first loop ");
  }
  while (!isCenterOnLine) {
    readSensors();
    Serial.println("stuck in second loop ");  
  }
  // moveForward();
  // delay(100);
}

void stopMotors() {
  analogWrite(eRightMotor, rightMotorSpeed);  // we choose a value from 0 to 255
  analogWrite(eLeftMotor, leftMotorSpeed);    // we choose a value from 0 to 255
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
}

void readSensors(){
  leftMostSensorValue = analogRead(leftMostSensorPin);
  leftSensorValue = analogRead(leftSensorPin);
  centerSensorValue = analogRead(centerSensorPin);
  rightSensorValue = analogRead(rightSensorPin);
  rightMostSensorValue = analogRead(rightMostSensorPin);

  isLeftMostOnLine = (leftMostSensorValue > sensorThreshold);
  isLeftOnLine = (leftSensorValue > sensorThreshold);
  isCenterOnLine = (centerSensorValue > sensorThreshold);
  isRightOnLine = (rightSensorValue > sensorThreshold);
  isRightMostOnLine = (rightMostSensorValue > sensorThreshold);
}
void applyPID() {
  for (int i = 0; i < 3; i++) {
    sensorValue[i] = analogRead(sensorPin[i]);
    if (sensorValue[i] > sensorThreshold) {
      sensorValue[i] = 1;
    } else {
      sensorValue[i] = 0;
    }
  }
    if (sensorValue[1] == 1) // left sensor 
    {
      currentWeight = 0.9;
    }    
    else  if (sensorValue[2] == 1) // right sensor 
    { 
      currentWeight = -0.9 ;
    }
    else if (sensorValue[0] == 1) // center sensor 
    {
      currentWeight = 0;
    }

  else 
  {
    currentWeight = 0;
  }

  
  error = idealSum - currentWeight;


  integral += error;
  derivative = error - previousError;
  previousError = error;

if (sensorValue[0] ==1) {
      integral= 0;
    }
  correction = kp * error + ki * integral + kd * derivative;

  rightMotorSpeed = baseSpeed - correction;
  leftMotorSpeed = baseSpeed + correction;
  // Serial.print("leftMotorSpeed= ");
  // Serial.print(leftMotorSpeed);
  // Serial.print("     rightMotorSpeed= ");
  // Serial.println(rightMotorSpeed);

  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(eLeftMotor, constrain(leftMotorSpeed , 0 , 255));
  analogWrite(eRightMotor, constrain (rightMotorSpeed , 0 , 255));
  Serial.println("PID IS HERE");

}


void makeDicision() {
  if (isLeftMostOnLine )  //every turn left is here 1XXX
  {
    turnLeft();//there are changes inside it
    if(millis() - prevMillis > 1800) // added this to not take more than one dicision at a time (helpful in path shortning)
    {
      Serial.println("dicision left");
      path = path + "L";
    } 
    prevMillis = millis();
  }

  if (!isLeftMostOnLine && isCenterOnLine && isRightMostOnLine)  //every go straight dicision is here exept the line follower one 01XX
  {
    moveForward();//there are changes inside it
    path = path + "S";
    Serial.println("dicision forward");
  }

  if (!isLeftMostOnLine && !isCenterOnLine && isRightMostOnLine)  // every turn right is here 001X
  {
    turnRight();//there are changes inside it
    if(millis() - prevMillis > 1800) // added this to not take more than one dicision at a time (helpful in path shortning)
    {
      Serial.println("dicision right");
      path = path + "R";
    } 
    prevMillis = millis();
  }

  if (!isLeftMostOnLine && !isCenterOnLine && !isRightMostOnLine && !isRightOnLine && !isLeftOnLine)  // dead end
  {
    moveForward(); // double check if it is just a dead angle 
    delay(300); // u can change this to find the most fitting one 
    if(!isLeftMostOnLine && !isCenterOnLine && !isRightMostOnLine && !isRightOnLine && !isLeftOnLine)
    {
    moveBackward();
    path = path + "B";
    Serial.println("dicision BACK");
    }
  }

  else{
    applyPID();
    // moveForward();
  }
}
