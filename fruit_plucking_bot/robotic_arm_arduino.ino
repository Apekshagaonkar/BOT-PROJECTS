//recieing the signal from the raspberrypi through the serial communication



#include <Servo.h>

char incoming = 'A';

Servo wrist; 
Servo elbow;
Servo finger; 

int posw = 0;
int pose = 0;
int posf =0;    
 

void setup() {
  Serial.begin(9600);
  
  wrist.attach(13);
  elbow.attach(11);
  finger.attach(12); 

  wrist.write(90);
  delay(100);
  elbow.write(0);
  delay(100);   
}

void loop() {
 if (Serial.available() > 0) {
    Serial.write("read to recieve\n");
    incoming = Serial.read();
    Serial.println(incoming,DEC);
    if(incoming == 'A'){
      posw=0;
      Serial.write(" find fruit");
      while( posw <= 90){//goes from 0 degrees to 180 degrees
                                            // in steps of 1 degree
         incoming= Serial.read();
         {if(incoming!='B')
           wrist.write(posw);              // tell servo to go to position in variable 'pos'
           delay(50);
           posw+=1;}
           incoming= Serial.read();
                                           // waits 15ms for the servo to reach the position
      }}
    else if(incoming=='B'){
      pose=0;
      Serial.write("pick");
      while( pose <= 5){
           elbow.write(pose);              // tell servo to go to position in variable 'pos'
           delay(100);
           pose+=1;
           }
      while( posf <= 80 ) {
           finger.write(posf);              // tell servo to go to position in variable 'pos'
           delay(15);
           posf+=1;
           }
    }
     else if(incoming=='C'){
            Serial.write("drop in box");
             pose=0;
             posf=0;
             posw=0;
             finger.write(posf);  
             wrist.write(posw);
             elbow.write(pose);             
          }
          
      }
 }
