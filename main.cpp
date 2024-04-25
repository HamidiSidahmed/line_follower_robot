// Definitionn of  pins of IR
const int leftSensorPin = A0;
const int centerSensorPin = A1;
const int rightSensorPin = A2;

// Definition pins of motor 
const int leftMotorPin1 = 2;
const int leftMotorPin2 = 3;
const int rightMotorPin1 = 4;
const int rightMotorPin2 = 5;

// Sensor threshold values (
const int sensorThreshold = 1000 ; // nkhyroha  7na nhar competition 3la jalt  black

void setup() {
  // Motor pins setup
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftSensorPin,INPUT);
  pinMode(centerSensorPin,INPUT);
  pinMode(rightSensorPin,INPUT);
  // Serial monetor for verification
  Serial.begin(9600);
}

void loop() {
  // Read ir values
  int leftSensorValue = analogRead(leftSensorPin);
  int centerSensorValue = analogRead(centerSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);

  //output reading
  Serial.print("Left: ");
  Serial.print(leftSensorValue);
  Serial.print(" Center: ");
  Serial.print(centerSensorValue);
  Serial.print(" Right: ");
  Serial.println(rightSensorValue);

  // logic of working of ir
  boolean isLeftOnLine = (leftSensorValue > sensorThreshold);
  boolean isCenterOnLine = (centerSensorValue > sensorThreshold);
  boolean isRightOnLine = (rightSensorValue > sensorThreshold);

  // Line following 
  if (!isLeftOnLine && !isCenterOnLine && !isRightOnLine) {
    // No line detected - stop (finish) (1 st preiority )
    stopMotors();
  } else if (!isLeftOnLine && isCenterOnLine && !isRightOnLine) {
    // Go straight (2nd preiority )
    goForward();
  } else if (isLeftOnLine && !isCenterOnLine && !isRightOnLine) {
    // go left 
    turnLeft();
  } else if (!isLeftOnLine && !isCenterOnLine && isRightOnLine) {
    // go right
    turnRight();
  } else {
    //  stop
    stopMotors();
  }
}

void goForward() {
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}

void turnLeft() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}

void turnRight() {
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
}

void stopMotors() {
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
}


