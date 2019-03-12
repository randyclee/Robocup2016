#include <SPI.h>  
#include <PixyI2C.h>
#include <Wire.h>
#include <NewPing.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define FTRIGGER_PIN  38  // FRONT// Arduino pin tied to trigger pin on the ultrasonic sensor.
#define FECHO_PIN     39 
#define LTRIGGER_PIN  48 //left
#define LECHO_PIN     49
#define BTRIGGER_PIN  44 //back
#define BECHO_PIN     45
#define RTRIGGER_PIN  32 //right 
#define RECHO_PIN     33

#define MAX_DISTANCE  200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

PixyI2C pixy;

int compassAddress = 0x21;       // From datasheet compass address is 0x42
int compassDirection; // tells me what way bot is facing

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *FL = AFMS.getMotor(4);
Adafruit_DCMotor *FR = AFMS.getMotor(1);
Adafruit_DCMotor *BL = AFMS.getMotor(3);
Adafruit_DCMotor *BR = AFMS.getMotor(2);

int mSpeed=200;
  int mSpeed2=200;
  int mSpeed0=200; 
  int compassTurnSpeed = 50;

NewPing Fsonarright(FTRIGGER_PIN, FECHO_PIN, MAX_DISTANCE);// NewPing setup of pins and maximum distance.
NewPing Lsonarright(LTRIGGER_PIN, LECHO_PIN, MAX_DISTANCE);
NewPing Bsonarright(BTRIGGER_PIN, BECHO_PIN, MAX_DISTANCE);
NewPing Rsonarright(RTRIGGER_PIN, RECHO_PIN, MAX_DISTANCE);


void setup()
{
  direction(true);
  Wire.begin();
  Serial.begin(9600);
  pixy.init();
 direction(true);
  
  AFMS.begin();  // create with the default frequency 1.6KHz
 //set the speed to midway min speed is 0 and max is 255

  BL->setSpeed(mSpeed);
  BR->setSpeed(mSpeed);
  BL->run(RELEASE);      // stopped
  BR->run(RELEASE);      // stopped
  FL->setSpeed(mSpeed);
  FR->setSpeed(mSpeed);
  FL->run(RELEASE);      // stopped
  FR->run(RELEASE);      // stopped 
}
void loop()
{ 
  compassDirection=direction(false);  
  Serial.print("COMPASS   ");
  Serial.println(compassDirection);
  
  
  static int i = 0;
  int j;

  uint16_t blocks;
  blocks = pixy.getBlocks();
   int dfront= Fsonarright.ping_cm();
  pinMode(FECHO_PIN,OUTPUT);
  digitalWrite(FECHO_PIN,LOW);
  pinMode(FECHO_PIN,INPUT);
      


    Serial.print("front: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dfront);
  Serial.println("cm"); 
      
  int dleft=  Lsonarright.ping_cm();
  pinMode(LECHO_PIN,OUTPUT);
  digitalWrite(LECHO_PIN,LOW);
  pinMode(LECHO_PIN,INPUT);
      
  Serial.print("left: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dleft);
  Serial.println("cm"); 
      
  int dback=  Bsonarright.ping_cm();
  pinMode(BECHO_PIN,OUTPUT);
  digitalWrite(BECHO_PIN,LOW);
  pinMode(BECHO_PIN,INPUT);
      
  Serial.print("back: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dback); 
  Serial.println("cm"); 
      
  int dright= Rsonarright.ping_cm();
  pinMode(RECHO_PIN,OUTPUT);
  digitalWrite(RECHO_PIN,LOW);
  pinMode(RECHO_PIN,INPUT);
      
  Serial.print("right: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dright);
  Serial.println("cm");
  
  
  //if you take the ball of, is it still 0?
  
  pixy.blocks[j].print();
  
  if (compassDirection>185){
    motorsTurnLeft();
    delay(20);
  motorsStop();  
}
  else if(compassDirection<176){
    motorsTurnRight();
    delay(20);
  motorsStop();  
  }
  else{
      /*if(dleft<40&&dleft>0){
        motorsMoveRight();
      }
      else if(dright<40&&dright>0){
        motorsMoveLeft();
      }*/
   /*if (dfront < 40 && dleft <35 && dright < 35){
    motorsBackward();
    Serial.println ("back");
  }
  else if (dback < 40 && dright <35 && dleft < 35){
    motorsForward();
    Serial.println ("forward");
  }
  else if(dback<40&&dback>0){
    motorsForward();
  }*/
  //elseif
  if (blocks&&pixy.blocks[i].signature==3)
  {
    if(dleft<30&&dleft>0){
      motorsMoveRight();
    }
    else if(dright<30&&dright>0){
      motorsMoveLeft();
    }
    
 }
    else{
  if (blocks&&pixy.blocks[i].signature==2)
  {
    if(dfront<20&&dfront>0){
      if(dleft<30){
        motorsMoveRight();
      }
      else if(dright<30){
        motorsMoveLeft();
        }
        else if(dright>30&&dleft>30) {
      motorsMoveLeft();
    }
    }    
  }
 if (blocks&&pixy.blocks[i].signature==1)
  {
      for (j=0; j<blocks; j++)
     { 
      if(pixy.blocks[0].x>120&&pixy.blocks[0].x<200){  
        motorsForward();
      }
      else if(pixy.blocks[0].x<120){
        motorsMoveLeft();
      }
      else if(pixy.blocks[0].x>200){
        motorsMoveRight();
      }
      else {
        motorsBackward();
      }
    }
  }
 }
} 
}

int direction(boolean zero) {
  static int offSet;
  int reading=0;
        
  if (zero) {
    // step 1: instruct sensor to read echoes 
      Wire.beginTransmission(compassAddress);  // transmit to device
                               // the address specified in the datasheet is 66 (0x42) 
                               // but i2c adressing uses the high 7 bits so it's 33 
      Wire.write('A');          // command sensor to measure angle  
      Wire.endTransmission();  // stop transmitting 
     
      // step 2: wait for readings to happen 
      delay(10);               // datasheet suggests at least 6000 microseconds 
      
      // step 3: request reading from sensor 
      Wire.requestFrom(compassAddress, 2);  // request 2 bytes from slave device #33 
      
      // step 4: receive reading from sensor 
      int reading=0;
         
      if(2 <= Wire.available())     // if two bytes were received 
      { //note: receive has been replaced by read
        reading = Wire.read();   // receive high byte (overwrites previous reading) 
        reading = reading << 8;     // shift high byte to be high 8 bits 
        reading += Wire.read();  // receive low byte as lower 8 bits 
        reading /= 10;
      }
      offSet=180-reading;    
  }
  else {
    // step 1: instruct sensor to0);
  BL->run(BACKWARD);
  FL->run(FORWARD);
  BR->run(BACKWARD);
  FR->run(FORWARD);
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  Serial.println("RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT RIGHT");
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
}
void motorsStop() { // method to stop
  BL->run(RELEASE);
  FL->run(RELEASE);
  BR->run(RELEASE);
  FR->run(RELEASE);
}

void motorsForwardLeft() {
  BL->run(FORWARD);
  FL->run(RELEASE);
  BR->run(RELEASE);
  FR->run(BACKWARD);
}

void motorsForwardRight() {
  BL->run(RELEASE);
  FL->run(FORWARD);
  BR->run(BACKWARD);
  FR->run(RELEASE);
}
