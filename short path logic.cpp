#include <Arduino.h>

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