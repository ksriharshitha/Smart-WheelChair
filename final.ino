#define m11 11    // rear motor
#define m12 12
#define m21 10    // front motor
#define m22 9
#include <SoftwareSerial.h>
#include <NewPing.h>
#define TRIGGER_PIN  3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     4  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN3 7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN3     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN4  5  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN4     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
int count[4];
int GotPath;

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
int count_bluetooth=0;
int TriggerThreshold=2;
int encoder_pin = 2; // pulse output from the module
unsigned int rpm; // rpm reading
float circum;
float distance;
float Adistance=0.0;
volatile byte pulses; // number of pulses
volatile byte Apulses=0; // number of pulses
unsigned long timeold;
unsigned long Atimeold=0;
unsigned long timeTurn;
int flagDest;
int travel_Flag;
// number of pulses per revolution
// based on your encoder disc
unsigned int pulsesperturn = 12;

#define x A3
#define y A4
#define z A5

int xsample=0;
int ysample=0;
int zsample=0;

#define samples 10

#define minVal -50
#define MaxVal 50

int mode[3];

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE); // Left
NewPing sonar3(TRIGGER_PIN4, ECHO_PIN4, MAX_DISTANCE); // Right
char str[2],i;
char playvar;
float directions[4];
float destination[4];
int Flag[4];

void forward()
{
   Serial.println("Forward");
   digitalWrite(m11, LOW);
   digitalWrite(m12, HIGH);
   digitalWrite(m21, LOW);
   digitalWrite(m22, HIGH);
}
void backward()
{
   Serial.println("Backward");
   digitalWrite(m11, HIGH);
   digitalWrite(m12, LOW);
   digitalWrite(m21, HIGH);
   digitalWrite(m22, LOW); 
}
void Stop()
{
   Serial.println("Stop");
   digitalWrite(m11, LOW);
   digitalWrite(m12, LOW);
   digitalWrite(m21, LOW);
   digitalWrite(m22, LOW);
}
void Actright()
{
   Stop();
   delay(1000);
   Serial.println("Right");
   digitalWrite(m11, LOW);
   digitalWrite(m12, LOW);
   digitalWrite(m21, LOW);
   digitalWrite(m22, HIGH);
   delay(1500);
   Stop();
}
void Actleft()
{
   Stop();
   delay(1000);
   Serial.println("Left");
   digitalWrite(m11, LOW);
   digitalWrite(m12, HIGH);
   digitalWrite(m21, LOW);
   digitalWrite(m22, LOW);
   delay(1700);
}
void right()
{
   Stop();
   delay(1000);
   Serial.println("Right");
   digitalWrite(m11, LOW);
   digitalWrite(m12, LOW);
   digitalWrite(m21, LOW);
   digitalWrite(m22, HIGH);
   delay(1400);
}
void left()
{
   Stop();
   delay(1400);
   Serial.println("Left");
   digitalWrite(m11, LOW);
   digitalWrite(m12, HIGH);
   digitalWrite(m21, LOW);
   digitalWrite(m22, LOW);
   delay(2000);
}
void TurnLeft(){
  if(Flag[0]==1){
    Flag[3]=1;
    Flag[0]=0;
    Flag[1]=0;
    Flag[2]=0;
  }
  else if(Flag[1]==1){
    Flag[2]=1;
    Flag[0]=0;
    Flag[1]=0;
    Flag[3]=0;
  }
  else if(Flag[2]==1){
    Flag[0]=1;
    Flag[0]=0;
    Flag[1]=0;
    Flag[2]=0;
  }
  else if(Flag[3]==1){
    Flag[1]=1;
    Flag[0]=0;
    Flag[2]=0;
    Flag[3]=0;
  }
}
void TurnRight(){
  if(Flag[0]==1){
    Flag[2]=1;
    Flag[0]=0;
    Flag[1]=0;
    Flag[3]=0;
  }
  else if(Flag[1]==1){
    Flag[3]=1;
    Flag[0]=0;
    Flag[1]=0;
    Flag[2]=0;
  }
  else if(Flag[2]==1){
    Flag[1]=1;
    Flag[0]=0;
    Flag[2]=0;
    Flag[3]=0;
  }
  else if(Flag[3]==1){
    Flag[0]=1;
    Flag[1]=0;
    Flag[2]=0;
    Flag[3]=0;
  }
}
void Turn180(){
  TurnRight();
  TurnRight();
}

//Position => Forward-1,Left-2,Right-3,Backward-4
bool checkUltraSonic(int position){
  if(position==1){
    if(count[0]<TriggerThreshold) {
    
      delay(1000);    //ping every second
      unsigned int dist_cms = sonar.ping_cm();
    
      if(dist_cms >=1 && dist_cms <=40) {
        count[0]++;
      }
      else if(count[0]>0){
        count[0]--;
      }
      return false;
    }
    else{
      count[0]=0;
      Serial.println("Obstacle in Front");
      return true;
    }
  }
  else if(position==2){
    if(count[1]<TriggerThreshold-1) {
    
      delay(1000);    //ping every second
      unsigned int dist_cms = sonar2.ping_cm();
    
      if(dist_cms >=1 && dist_cms <=40) {
        count[1]++;
      }
      else if(count[1]>0){
        count[1]--;
      }
      Serial.print("distance at left:");
      Serial.println(dist_cms);
      return false;
    }
    else{
      count[1]=0;
      Serial.println("Obstacle at Left");
      return true;
    
    }    
  }
  else if(position==3){
    if(count[2]<TriggerThreshold-1) {
    
      delay(1000);    //ping every second
      unsigned int dist_cms = sonar3.ping_cm();
    
      if(dist_cms >=1 && dist_cms <=40) {
        count[2]++;
      }
      else if(count[2]>0){
        count[2]--;
      }
      Serial.print("distance at right:");
      Serial.println(dist_cms);
      return false;
    }
    else{
      count[2]=0;
      Serial.println("Obstacle at Right");
      return true;
    }
  }
  else if(position==4){
    if(count[3]<TriggerThreshold) {
    
      delay(1000);    //ping every second
      unsigned int dist_cms = 0.0;//sonar4.ping_cm();
    
      if(dist_cms >=1 && dist_cms <=65) {
        count[3]++;
      }
      else if(count[3]>0){
        count[3]--;
      }
      return false;
    }
    else{
      count[3]=0;
      Serial.println("Obstacle at Back");
      return true;
    }
  }
}
bool ObstacleCheck() {
  if(GotPath<-1){
    if((millis()-timeTurn) > 5000){
      Stop();
      if(GotPath==-2){
        Turn180();
        left();
        left();
        GotPath=-4;
        timeTurn = millis();
      }
      else if(GotPath==-3){
        Turn180();
        right();
        right();
        GotPath=-5;
        timeTurn = millis();
      }
      else if(GotPath==-4){
        TurnLeft();
        left();
        GotPath=-1;
        return false;
      }
      else if(GotPath==-5){
        TurnRight();
        right();
        GotPath=2;
        return false;
      }
    }
    else if(GotPath==-2){
      if(!checkUltraSonic(3)){
        Serial.println("Checking obstacle in right");
        Stop();
        TurnRight();
        right();
        GotPath=1;
      }
    }
    else if(GotPath==-3){
      if(!checkUltraSonic(2)){
        Stop();
        TurnLeft();
        left();
        GotPath=1;
      }
    }
  }
  else if(checkUltraSonic(1)){
    Stop();
    if(checkUltraSonic(2)){
      GotPath=0;
    }
    else if(GotPath==-1 || GotPath!=2){
      TurnLeft();
      left();
      GotPath=-2;
      timeTurn = millis();
    }
    if(GotPath==0 || GotPath==-1){
     if(checkUltraSonic(3)){
        GotPath=0;
      }
      else{
        TurnRight();
        right();
        GotPath=-3;
        timeTurn = millis();
      } 
    }
    if(GotPath==0 || GotPath==2){
      Stop();
      return false;
    }
  }
  return true;  
}

void counter()
{
   //Update count
   pulses++;
}

void Acounter()
{
   //Update count
   Apulses++;
}

void tilt_forward()
{
   Serial.println("Forward");
   digitalWrite(m11, LOW);
   digitalWrite(m12, HIGH);
   digitalWrite(m21, LOW);
   digitalWrite(m22, HIGH);
}

void tilt_backward()
{
   Serial.println("Backward");
   digitalWrite(m11, HIGH);
   digitalWrite(m12, LOW);
   digitalWrite(m21, HIGH);
   digitalWrite(m22, LOW); 
}

void tilt_Stop()
{
   Serial.println("Stop");
   digitalWrite(m11, LOW);
   digitalWrite(m12, LOW);
   digitalWrite(m21, LOW);
   digitalWrite(m22, LOW);
}

void tilt_left()
{
   Serial.println("Left");
   digitalWrite(m11, LOW);
   digitalWrite(m12, LOW);
   digitalWrite(m21, LOW);
   digitalWrite(m22, HIGH);
}

void tilt_right()
{
   Serial.println("Right");
   digitalWrite(m11, LOW);
   digitalWrite(m12, HIGH);
   digitalWrite(m21, LOW);
   digitalWrite(m22, LOW);
}


void forward_bluetooth()
{
   digitalWrite(m11, LOW);
   digitalWrite(m12, HIGH);
   digitalWrite(m21, LOW);
   digitalWrite(m22, HIGH);
}

void backward_bluetooth()
{
   digitalWrite(m11, HIGH);
   digitalWrite(m12, LOW);
   digitalWrite(m21, HIGH);
   digitalWrite(m22, LOW); 
}

void Stop_bluetooth()
{
   digitalWrite(m11, LOW);
   digitalWrite(m12, LOW);
   digitalWrite(m21, LOW);
   digitalWrite(m22, LOW);
}

void left_bluetooth()
{
   digitalWrite(m11, LOW);
   digitalWrite(m12, LOW);
   //delay(100);
   digitalWrite(m21, LOW);
   digitalWrite(m22, HIGH);
   delay(600);
   Stop();
}

void right_bluetooth()
{
   digitalWrite(m11, LOW);
   digitalWrite(m12, HIGH);
   //delay(100);
   digitalWrite(m21, LOW);
   digitalWrite(m22, LOW);
   delay(600);
   Stop();
}


void bluetooth_ObstacleCheck() {

  if(count_bluetooth<TriggerThreshold) {
    
    delay(1000);    //ping every second
    unsigned int dist_cms = sonar.ping_cm();
  
    if(dist_cms >=1 && dist_cms <=40) {

       count_bluetooth++;
    }
    else if(count_bluetooth>0){

      count_bluetooth--;
    }
  }
  else{
    count_bluetooth=0;
    Stop();
    //left();
    backward_bluetooth();
    delay(1000);
    Stop();
  }  
}

void bluetooth(char actionkey) 
{
  bluetooth_ObstacleCheck();
  
    Serial.println(actionkey);
    if(str[i-1]=='w')
    {
     Serial.println("Forward");
     forward_bluetooth();
     playvar=str[i-1];
     i=0;
    }

    else if(str[i-1]=='a')
    {
     Serial.println("Left");
     right_bluetooth();
     playvar=str[i-1];
     i=0;
    }

    else if(str[i-1]=='d')
    {
      Serial.println("Right");
      left_bluetooth();
      playvar=str[i-1];
      i=0;
    }
    
    else if(str[i-1]=='s')
    {
      Serial.println("Backward");
      backward_bluetooth();
      playvar=str[i-1];
      i=0;
    }

    else if(str[i-1]=='z')
    {
      Serial.println("Stop");
      Stop();
      i=0;
    }

    else if(str[i-1]=='x')
    {
      Serial.println("Start");
       if(playvar=='w')
      {
       Serial.println("Forward");
       forward_bluetooth();
       i=0;
      }
    
      else if(playvar=='a')
      {
       Serial.println("Left");
       right_bluetooth();
       i=0;
      }
    
      else if(playvar=='d')
      {
        Serial.println("Right");
        left_bluetooth();
        i=0;
      }
      
      else if(playvar=='s')
      {
        Serial.println("Backward");
        backward_bluetooth();
        i=0;
      }  
    }
}

void tilt() 
{
    int value1=analogRead(x);
    int value2=analogRead(y);
    int value3=analogRead(z);

    int xValue=xsample-value1;
    int yValue=ysample-value2;
    int zValue=zsample-value3;
    if(xValue >= 35 && xValue <=45 && yValue >= 40 && yValue <=50){
      Serial.println("stop");
      tilt_Stop();
    }
    else if(xValue<=10){
      Serial.println("left");
      tilt_right();
    }
    else if(xValue>=72){
      Serial.println("right");
      tilt_left();
    }
    else if(yValue>=72){
      Serial.println("front");
      tilt_forward();
    }
    else if(yValue<=10){
      Serial.println("back");
      tilt_backward();
    }
    
    Serial.print("x=");
    Serial.println(xValue);
    Serial.print("y=");
    Serial.println(yValue);
    Serial.print("z=");
    Serial.println(zValue);
    delay(2000);
}

void auto_navigate() 
{
  if(flagDest==0){
    if(ObstacleCheck()){
      Serial.print("GotPath:");
      Serial.println(GotPath);
      forward();
    }
    if (millis() - timeold >= 1000) {
        //Don't process interrupts during calculations
        detachInterrupt(0);
        rpm = (60*1000 / pulsesperturn )/ (millis() - timeold)* pulses;
        circum = 2.0* 3.141592* 3.0;
        if(rpm>0){
          distance += circum;//*(rpm/5.0);
          if(Flag[0]==1){
            if(directions[1] > 0){
              directions[1]-=1.0;
            }
            else{
              
              directions[0]+=1.0;
            }
          }
          else if(Flag[1]==1){
            if(directions[0] > 0){
              directions[0]-=1.0;
            }
            else{
              
              directions[1]+=1.0;
            }
          }
          else if(Flag[2]==1){
            if(directions[3] > 0){
              directions[3]-=1.0;
            }
            else{
              
              directions[2]+=1.0;
            }
          }
          else if(Flag[3]==1){
            if(directions[2] > 0){
              directions[2]-=1.0;
            }
            else{
              
              directions[3]+=1.0;
            }
          }
        }
        timeold = millis();
        pulses = 0;
        //Restart the interrupt processing
        attachInterrupt(0, counter, FALLING);
    }
  }
  if(((directions[0] < destination[0]+1.0) && (directions[0] > destination[0]-1.0)) && ((directions[2] < destination[2]+1.0) && (directions[2] > destination[2]-1.0))){
    Serial.println("You reached your destination!!!");
    flagDest=1;
    Stop();
  }
  else if((directions[0] < destination[0]+1.0) && (directions[0] > destination[0]-1.0) && travel_Flag==0){
    Serial.println("You have covered your North distance.Travelling East.");
    Stop();
    TurnRight();
    Actright();
    travel_Flag=1;
  }
  else if((directions[2] < destination[2]+1.0) && (directions[2] > destination[2]-1.0) && travel_Flag==1){
    Serial.println("You have covered your East distance.Travelling North.");
    Stop();
    TurnLeft();
    Actleft();
    travel_Flag=0;
  }
}

void setup() 
{
  Serial.begin(9600);
  pinMode(m11, OUTPUT);
  pinMode(m12, OUTPUT);
  pinMode(m21, OUTPUT);
  pinMode(m22, OUTPUT);
  pinMode(encoder_pin, INPUT);
   //Interrupt 0 is digital pin 2
   //Triggers on Falling Edge (change from HIGH to LOW)
   attachInterrupt(0, counter, FALLING);
   // Initialize
   pulses = 0;
   rpm = 0;
   timeold = 0;
   distance=0.0;
   mode[0]=1;
   mode[1]=0;
   mode[2]=0;

   int i;
   for(i=0;i<samples;i++)
  {
    xsample+=analogRead(x);
    ysample+=analogRead(y);
    zsample+=analogRead(z);
  }

  xsample/=samples;
  ysample/=samples;
  zsample/=samples;

   for(i=0;i<4;i++){
    directions[i]=0.0;
    destination[i]=0.0; 
    Flag[i]=0;
    count[i]=0;
   }

   GotPath=-1;
   destination[0]=5.0;
   destination[2]=5.0;
   flagDest=0;
   //Assuming Facing North
   Flag[0]=1;
   if(destination[1]>0.0 && destination[2]>0.0){
     Stop();
     TurnRight();
     right();
   }
   else if(destination[1]>0.0 && destination[3]>0.0){
     Stop();
     Turn180();
     right();
     right();
   }
   else if(destination[0]>0.0 && destination[3]>0.0){
     Stop();
     TurnLeft();
     left();
   }
   travel_Flag=0;
}

void loop(){

  while(Serial.available())
  {
    char ch=Serial.read();
    str[i++]=ch;

    if(str[i-1]=='1'){

      setup();
      mode[0]=1;
      mode[1]=0;
      mode[2]=0;
      i=0;      
    }
    else if(str[i-1]=='2'){

      setup();
      mode[0]=0;
      mode[1]=1;
      mode[2]=0;
      i=0;
    }
    else if(str[i-1]=='3'){

      setup();
      mode[0]=0;
      mode[1]=0;
      mode[2]=1;      
      i=0;
    }

    if(mode[0]==1){

      if(str[i-1]!='1'){

        bluetooth(str[i-1]); 
      }
      i=0;   
    }
    if(mode[1]==1){
      tilt();
      i=0;
    }
    if(mode[2]==1){
      
      auto_navigate();
      i=0;
    }
  }
  if(mode[1]==1){

    tilt();
    i=0;
  }
  else if(mode[2]==1){

    auto_navigate();
    i=0;
  }
  
}

