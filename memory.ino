int btnPin[4];
int ledPin[4];
int readBtn[4];
int oldPin;
int Ai;
int dt = 1000;
int i = 0 ;
void setup() {
for (int i = 0 ; i <4 ; i++){
  Serial.begin(9600);
  btnPin[i]=i+2;
  ledPin[i]=i+6;
  pinMode(btnPin[i], INPUT);
  pinMode(ledPin[i], OUTPUT);
  digitalWrite(btnPin[i], HIGH);
    readBtn[i] = digitalRead(btnPin[i]);
}

}

void loop() {
readBtn[i]=1;
Ai = random(0,4);
digitalWrite(ledPin[Ai] , HIGH);
delay(dt);
digitalWrite(ledPin[Ai], LOW);

  while(readBtn[i]==1)
  {
    i++;  
    if(i>3)
    {
      i=0;
    }
    delay(50);
    readBtn[i]=digitalRead(btnPin[i]);
  }


  if(readBtn[i]==0 && i == Ai )
  {
    digitalWrite(ledPin[i], HIGH);
    delay(dt);
    digitalWrite(ledPin[i], LOW);
  }else
  {
    for(i=0 ; i<4 ; i++){
      digitalWrite(ledPin[i], HIGH);
    }
  }

}


