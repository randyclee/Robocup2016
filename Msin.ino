  
//Note: the Adafruit motorshield libary V2 must be put in the Arduino library folder,
//usually at C:\Program Files (x86)\Arduino\libraries

#include <Wire.h> 
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <NewPing.h>
#include <QTRSensors.h>

int irValues[6];

#define NUM_SENSORS   4     
#define TIMEOUT       2500

#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
//ORANGE PURPLE IS TRIG 
//RED GREY IS ECHO
#define FTRIGGER_PIN  37  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define FECHO_PIN     36  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define RTRIGGER_PIN  35  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define RECHO_PIN     34  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define BTRIGGER_PIN  33   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define BECHO_PIN     32  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define LTRIGGER_PIN  31   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define LECHO_PIN     30  // Arduino pin tied to echo pin on the ultrasonic sensor.

NewPing sonarfront(FTRIGGER_PIN, FECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarright(RTRIGGER_PIN, RECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarback(BTRIGGER_PIN, BECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarleft(LTRIGGER_PIN, LECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *FMotor = AFMS.getMotor(2);
Adafruit_DCMotor *RMotor = AFMS.getMotor(3);
Adafruit_DCMotor *BMotor = AFMS.getMotor(4);
Adafruit_DCMotor *LMotor = AFMS.getMotor(1);


int mRight, mLeft, mFront, mBack;
int compassAddress = 0x21; 
int compassDirection;


QTRSensorsRC qtrrc((unsigned char[]) {38, 39, 40, 41},
NUM_SENSORS, TIMEOUT, QTR_NO_EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

int BLQTR, BRQTR, FLQTR, FRQTR, BLQTR1, BRQTR1, FLQTR1, FRQTR1=0;


int sensorValue;
const int irSensor = 15;

void setup()
{
  Wire.begin();                // join i2c bus (address optional for master)  
  Serial.begin(9600);
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  mRight=mLeft=mBack=mFront=130; //set the speed to midway min speed is 0 and max is 255
  FMotor->setSpeed(mRight);
  RMotor->setSpeed(mLeft);
  BMotor->setSpeed(mFront);
  LMotor->setSpeed(mBack);
  
  compassDirection=direction(true);
  
  FMotor->run(RELEASE);      // stopped
  RMotor->run(RELEASE);      // stopped  
  BMotor->run(RELEASE);      // stopped
  LMotor->run(RELEASE);      // stopped  
  //Serial.begin(9600);
  
  qtrrc.read(sensorValues);
  BLQTR1 = sensorValues[1];
  BRQTR1 = sensorValues[0];
  FLQTR1 = sensorValues[2];
  FRQTR1 = sensorValues[3];
    
   //Wire.begin(); 
  irReadings(true);
}


void loop() {
  compassDirection=direction(false); 
  
  qtrrc.read(sensorValues);
  
   qtrrc.read(sensorValues);

  BLQTR = sensorValues[1];
  BRQTR = sensorValues[0];
  FLQTR = sensorValues[2];
  FRQTR = sensorValues[3];
  
  unsigned char i;
  
  sensorValue = analogRead(irSensor);
  Serial.print("sensor value is: ");
  Serial.println(sensorValue);
  
  
  BLQTR = sensorValues[1];
  BRQTR = sensorValues[0];
  FLQTR = sensorValues[2];
  FRQTR = sensorValues[3];
  

  int dfront= sonarfront.ping_cm();//US_ROUNDTRIP_CM);
  
    //to correct zeros when d
  pinMode(FECHO_PIN,OUTPUT);
  digitalWrite(FECHO_PIN,LOW);
  pinMode(FECHO_PIN,INPUT);
  
  Serial.print("front: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dfront);
  Serial.println("cm");
  
  
  //right
                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int dright= sonarright.ping_cm();//US_ROUNDTRIP_CM);
  
    //to correct zeros when d
  pinMode(RECHO_PIN,OUTPUT);
  digitalWrite(RECHO_PIN,LOW);
  pinMode(RECHO_PIN,INPUT);
  
  Serial.print("right: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dright);
  Serial.println("cm");
  
  
  //back 
  int dback= sonarback.ping_cm();//US_ROUNDTRIP_CM);


    //to correct zeros when d
  pinMode(BECHO_PIN,OUTPUT);
  digitalWrite(BECHO_PIN,LOW);
  pinMode(BECHO_PIN,INPUT);
  
  Serial.print("back: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dback);
  Serial.println("cm"); 
  
    //left 
  int dleft= sonarleft.ping_cm();//US_ROUNDTRIP_CM);

    //to correct zeros when d
  pinMode(LECHO_PIN,OUTPUT);
  digitalWrite(LECHO_PIN,LOW);
  pinMode(LECHO_PIN,INPUT);
  
  Serial.print("left: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dleft);
  Serial.println("cm");   


  irReadings(false);

  Serial.print("compass direction: ");
  Serial.println(compassDirection);
  for (int i=0; i<6; i++) {
    Serial.println(irValues[i]);
  }
  if(compassDirection >= 190){
     mRight=mLeft=mBack=mFront=80;
     motorsLeft();
    Serial.println("turn left");
  }
  else if (compassDirection <= 170){
     mRight=mLeft=mBack=mFront=80;
     motorsRight();
    Serial.println("turn right"); 
  }
  if(dleft<50&&dleft>0){
          motorsSlideRight();
        }
  else if(dright<50&&dright>0){
         motorsSlideLeft();
  }
   else if(abs(FLQTR-FLQTR1)>1000){
     mRight=mLeft=mBack=mFront=150;
             FMotor->setSpeed(mRight);
            RMotor->setSpeed(mLeft);
            BMotor->setSpeed(mFront);
            LMotor->setSpeed(mBack);
      motorsBackward(); 
   } 
   else if (abs(FRQTR-FRQTR1)>1000){
     mRight=mLeft=mBack=mFront=150;
             FMotor->setSpeed(mRight);
            RMotor->setSpeed(mLeft);
            BMotor->setSpeed(mFront);
            LMotor->setSpeed(mBack);
      motorsBackward(); 
   } 
   else if (sensorValue > 550){
             mRight=mLeft=mBack=mFront=150;
             FMotor->setSpeed(mRight);
            RMotor->setSpeed(mLeft);
            BMotor->setSpeed(mFront);
            LMotor->setSpeed(mBack);
            motorsForward(); 
       }

        else if (irValues[0]==7||irValues[0]==6){
          mRight=mLeft=mBack=mFront=120;
           FMotor->setSpeed(mRight);
          RMotor->setSpeed(mLeft);
          BMotor->setSpeed(mFront);
          LMotor->setSpeed(mBack);
          motorsSlideLeft();
        }
        else if (irValues[0]==3||irValues[0]==4){
          mRight=mLeft=mBack=mFront=120;
           FMotor->setSpeed(mRight);
            RMotor->setSpeed(mLeft);
            BMotor->setSpeed(mFront);
            LMotor->setSpeed(mBack);
           motorsSlideRight();
        }  
        else if(dback<15){
          motorsStop(); 
        }
        else{
           motorsBackward(); 
        }
   mRight=mLeft=mBack=mFront=60;
} 

void motorsForward() {
  Serial.println("Forward");
  LMotor->run(BACKWARD);      
  RMotor->run(BACKWARD); 
  BMotor->run(RELEASE);
  FMotor->run(RELEASE);   

}
void motorsDiagonalBL() {
  Serial.println("Diagonal bl");
  FMotor->run(BACKWARD);      
  BMotor->run(FORWARD);
  LMotor->run(FORWARD);   
  RMotor->run(FORWARD);
}
void motorsDiagonalBR() {
  Serial.println("Diagonal Br");
  LMotor->run(FORWARD);   
  RMotor->run(FORWARD);
  FMotor->run(FORWARD);      
  BMotor->run(BACKWARD);
}

void motorsBackward() {
  Serial.println("Backward");
  LMotor->run(FORWARD);   
  RMotor->run(FORWARD); 
  FMotor->run(RELEASE);   
  BMotor->run(RELEASE); 
 
}

void motorsStop() {
  Serial.println("stop");
  FMotor->run(RELEASE);      // stopped
  RMotor->run(RELEASE);      // stopped  
  BMotor->run(RELEASE);      // stopped
  LMotor->run(RELEASE);      // stopped         
}

void motorsSlideRight() {
  Serial.println("Slide Right");
  FMotor->run(FORWARD);      
  BMotor->run(BACKWARD);
  LMotor->run(RELEASE);
  RMotor->run(RELEASE);
}

void motorsSlideLeft() {
  Serial.println("Slide Left");
  FMotor->run(BACKWARD);      
  BMotor->run(FORWARD);
    LMotor->run(RELEASE);
  RMotor->run(RELEASE);
}

void motorsLeft() {
  Serial.println("Left");
  FMotor->run(BACKWARD);      
  BMotor->run(BACKWARD);
  RMotor->run(BACKWARD);      // stopped
  LMotor->run(BACKWARD);  // stopped   
}


void motorsRight() {
  Serial.println("Left");
  FMotor->run(FORWARD);      
  BMotor->run(FORWARD);
  RMotor->run(FORWARD);      // stopped
  LMotor->run(FORWARD);      // stopped   
}

void irReadings(boolean zero) {
  if (zero) {
    Wire.beginTransmission(0x08);
    Wire.write(0x41); 
    Wire.write(0x00);  //set AC modulated mode
    Wire.endTransmission();
  }
  else {
    Wire.beginTransmission(0x08);
    Wire.write(0x48); 
    Wire.write(0x06);  
    Wire.endTransmission();
    
    Wire.requestFrom(0x08, 6);  //read direction and strength values
    
    int c=0;
    while(Wire.available()) { // slave may send less than requested
      irValues[c] = Wire.read(); // receive a byte as character
      c++;
    }    
  }
}
  
 int direction(boolean zero) 
{
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
      if(2 <= Wire.available())     // if two bytes were received 
      { //note: receive has been replaced by read
        reading = Wire.read();   // receive high byte (overwrites previous reading) 
        reading = reading << 8;     // shift high byte to be high 8 bits 
        reading += Wire.read();  // receive low byte as lower 8 bits 
        reading /= 10;
      }
  }  
  
  if (reading+offSet<0)  //correction for too small/big numbers 
    return 360+(reading+offSet);
  else if (reading+offSet>360)
    return (reading+offSet)-360;
  else
   return reading+offSet;
}

/*

int pingCensors(){
  
  int dfront= sonarfront.ping_cm();//US_ROUNDTRIP_CM);
  
    //to correct zeros when d
  pinMode(FECHO_PIN,OUTPUT);
  digitalWrite(FECHO_PIN,LOW);
  pinMode(FECHO_PIN,INPUT);
  
  Serial.print("front: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dfront);
  Serial.println("cm");
  
  
  //right
                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int dright= sonarright.ping_cm();//US_ROUNDTRIP_CM);
  
    //to correct zeros when d
  pinMode(ECHO_PIN1,OUTPUT);
  digitalWrite(ECHO_PIN1,LOW);
  pinMode(ECHO_PIN1,INPUT);
  
  Serial.print("right: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dright);
  Serial.println("cm");
  
  
  //back 
   int dback= sonarback.ping_cm();//US_ROUNDTRIP_CM);

    //to correct zeros when d
  pinMode(BECHO_PIN,OUTPUT);
  digitalWrite(BECHO_PIN,LOW);
  pinMode(BECHO_PIN,INPUT);
  
  Serial.print("back: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dback);
  Serial.println("cm"); 
  
    //left 
   int dleft= sonarleft.ping_cm();//US_ROUNDTRIP_CM);

    //to correct zeros when d
  pinMode(LECHO_PIN,OUTPUT);
  digitalWrite(LECHO_PIN,LOW);
  pinMode(LECHO_PIN,INPUT);
  
  Serial.print("left: "); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  Serial.print(dleft);
  Serial.println("cm"); 
  
}
*/
