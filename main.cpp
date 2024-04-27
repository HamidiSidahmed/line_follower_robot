someone, [4/27/2024 12:57 PM]
//  IR  pins
const int leftMostSensorPin =A0;
const int leftSensorPin = A1;
const int centerSensorPin = A2;
const int rightSensorPin = A3;
const int rightMostSensorPin = A4;

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

//A counter for the steps number
int stepsNumber; // to record the number of the steps till the end

//A constant to maximize the size of the array
const int i=1000;
//A characters'array to record the path
char path[i]; 

//this one just to print the path ki nkounou ntestou mn b3d ngal3ah (later maybe ns79ouh bch ndirou shortest path)
int j;

void setup() {
  // Motor pins setup
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  
  // Serial communication for debugging
  Serial.begin(9600);

  //initialize the number of steps
  stepsNumber=0;
}

void loop() {
  
  // Read sensor values
  int leftMostSensorValue = analogRead(leftMostSensorPin);
  int leftSensorValue = analogRead(leftSensorPin);
  int centerSensorValue = analogRead(centerSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);
  int rightMostSensorValue = analogRead(rightMostSensorPin);
  
  // Debugging output
  Serial.print ("Left most:");
  Serial.print(leftMostSensorPin);
  Serial.print("Left : ");
  Serial.print(leftSensorPin);
  Serial.print(" Center: ");
  Serial.print(centerSensorValue);
  Serial.print(" Right : ");
  Serial.println(rightSensorPin);
  Serial.print ("right most:");
  Serial.print(rightMostSensorPin);

  // Determine the line following logic
  bool isLeftMostOnLine =(leftMostSensorValue > sensorThreshold);
  bool isLeftOnLine = (leftSensorValue > sensorThreshold);
  bool isCenterOnLine = (centerSensorValue > sensorThreshold);
  bool isRightOnLine = (rightSensorValue > sensorThreshold);
  bool isRightMostOnLine =(rightMostSensorValue > sensorThreshold);
  
  //determine when the end is found (all black)
  bool endIsFound = ( isLeftMostOnLine && isLeftOnLine && isCenterOnLine && isRightOnLine && isRightMostOnLine); // kmlt maze ila ga3 rahm ydetectiw noir

  // start solving the maze using the LSRB logic  (left straight right back)
  while(!endIsFound){ 
  //First priority to the left line   
     if( //only left line exists
       (!isLeftMostOnLine && isLeftOnLine && !isCenterOnLine && !isRightOnLine && !isRightMostOnLine) or //the near left detecter
       //both left and right lines exist'T'
       (!isLeftMostOnLine && isLeftOnLine && !isCenterOnLine && isRightOnLine && !isRightMostOnLine) or
       //intersection '+'
       (!isLeftMostOnLine && isLeftOnLine && isCenterOnLine && isRightOnLine && !isRightMostOnLine)or 
       //both staight and left lines exist 
       (!isLeftMostOnLine && isLeftOnLine && isCenterOnLine && !isRightOnLine && !isRightMostOnLine))
       {
        turnLeft();
       
        //store the direction in the array as left
        path[stepsNumber]='L';  
         
        //increment the number of steps
        stepsNumber++; 
        
       if(isCenterOnLine)
         { // it will stop turning until the central sensor detects black (90°)
          moveForward();
         } 
       }
       
       //Second priority to straight line
     if ( //only straight line exists '|'
       (!isLeftMostOnLine && !isLeftOnLine && isCenterOnLine && !isRightOnLine && !isRightMostOnLine)or
       //both staight and right lines exist 
       (!isLeftMostOnLine && !isLeftOnLine && isCenterOnLine && isRightOnLine && !isRightMostOnLine))
       {
         moveForward();

someone, [4/27/2024 12:57 PM]
//store the direction in the array as staight
         path[stepsNumber]='S';
                  
         //increment the number of steps
         stepsNumber++;
         
       }
     if (//only right line exists 
       !isLeftMostOnLine && !isLeftOnLine && !isCenterOnLine && isRightOnLine && !isRightMostOnLine)
       {
         turnRight(); 

         //store the direction in the array as right
         path[stepsNumber]='R';
                  
         //increment the number of steps
         stepsNumber++;
         

          if(isCenterOnLine)
         { // it will stop turning until the central sensor detects black (90°)
          moveForward(); 
         } 
       }
       if(// it reaches a dead end (all white)
        !isLeftMostOnLine && !isLeftOnLine && !isCenterOnLine && !isRightOnLine && !isRightMostOnLine)
        {
          //the difference here is that it will turn a 180° while before it was turning 90° only 
         turnRight(); 
  
         //store the direction in the array as staight
         path[stepsNumber]='B';
                  
          //increment the number of steps
         stepsNumber++;
       
         
         if(isCenterOnLine)
         { // it will stop turning until the central sensor detects black (180°)
          moveForward(); 
         } 
        }
        //IF the car is not perfectly  located in the centrer 
        //mn b3d nzidou two functions b la vitesse bch ydour fisa3 wla b chwi 3la 7ssab ida khrjt bzaf wla ghi chwi 
        if( //shifted to the left too much 
          isLeftMostOnLine && !isCenterOnLine )
          {
            turnLeft();
         if(isCenterOnLine)
         { // it will stop until the central sensor detects black (it gets back to its perfect position)
          moveForward();
         }
          }
        if( //shifted to the left a bit 
          isLeftOnLine && !isCenterOnLine )
          {
            turnLeft();
         if(isCenterOnLine)
         { // it will stop until the central sensor detects black (it gets back to its perfect position)
          moveForward();
         }
          }
         if( //shifted to the right too much 
          isRightMostOnLine && !isCenterOnLine )
          {
            turnRight();
         if(isCenterOnLine)
         { // it will stop until the central sensor detects black (it gets back to its perfect position)
          moveForward();
         }
         } 
         if( //shifted to the right a bit
          isRightOnLine && !isCenterOnLine )
          {
            turnRight();
         if(isCenterOnLine)
         { // it will stop until the central sensor detects black (it gets back to its perfect position)
          moveForward();
         }
          }
          
        //small delay for stability(we can remove it later if we saw that the car is stable without it)
       delay(100);
       }
  //end line is found 
  stopMotors();
  //printing the obtained path for testing 
  for (j=0; j<=stepsNumber-1; j++)
  { 
    Serial.print(j);
    Serial.println(path[j]);
    }
  for (j=0;j<=stepsNumber-1;j++)
  {
    if(path[j]=='B')
    {
      if(//LBR=B
        path[j-1]=='L'&&path[j+1]=='R')
      {
       path[j-1]='B';
      }
      
      if(//LBS=R
        path[j-1]=='L'&&path[j+1]=='S')
      {
       path[j-1]='R';
      }
      
      if(//RBL=B
        path[j-1]=='R'&&path[j+1]=='L')
      {
       path[j-1]='B';
      }
      
      if(//SBL=R
        path[j-1]=='S'&&path[j+1]=='L')
      {
       path[j-1]='R';
      }
      
      if(//SBS=B
        path[j-1]=='S'&&path[j+1]=='S')
      {
       path[j-1]='B';
      }
      
      if(//LBL=S
        path[j-1]=='L'&&path[j+1]=='L')
      {
       path[j-1]='S';
      }
    }
    
   //printing the optimized path for testing 
   for (j=0; j<=stepsNumber; j++)
  { 
    Serial.print(j);
    Serial.println(path[j]);
    }
  }
    
       
  }


//functions (mnb3d ndirohm as headers psq tkhlttna chwya kifch nriglohm fi headers plz chekina logic ila shih)
void moveForward() {
   analogWrite(eRightMotor, rightMotorSpeed -30); // we choose a value from 0 to 255 
  analogWrite(eLeftMotor, leftMotorSpeed-30); // we choose a value from 0 to 255
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}

void turnLeft() {
  analogWrite(eRightMotor, rightMotorSpeed ); // we choose a value from 0 to 255 
  analogWrite(eLeftMotor, leftMotorSpeed ); // we choose a value from 0 to 255
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
}

void turnRight() {
  analogWrite(eRightMotor, rightMotorSpeed ); // we choose a value from 0 to 255 
  analogWrite(eLeftMotor, leftMotorSpeed ); // we choose a value from 0 to 255
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
